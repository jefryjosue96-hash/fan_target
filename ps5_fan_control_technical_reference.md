# PS5 Fan Control: Technical Reference

This document describes the observed PS5 fan-control stack, the known
`/dev/icc_fan` ioctl interface, thermal monitoring APIs, and the constraints
that matter when changing the automatic servo target.

## 1. Control architecture

```text
SceShellCore / SystemStateMgr                 BAPM / application state
        |                                              |
        | selects servo pattern                        | publishes universal mode
        v                                              v
user process -> ioctl -> /dev/icc_fan -> ICC service -> Southbridge controller
                                                   |
                 board/vendor autoservo tables ----+
                                                   |
                    temperature feedback ----------+
                                                   v
                                             fan PWM duty
```

The observed control system has several separate layers:

1. **Board/vendor calibration.** The kernel selects a hardware-specific
   autoservo table based on board and memory-vendor information. The complete
   table is approximately 1710 bytes and is sent through ICC commands 10/14.
2. **Servo pattern.** ShellCore selects one of four preloaded pattern banks.
   Home and game transitions were observed as pattern 0 and pattern 1.
3. **Runtime autoservo parameters.** A 28-byte ioctl object contains the active
   target temperature and other controller parameters.
4. **Manual duty.** A separate register can be read or written, but it is not
   the same as actual duty and does not constitute a complete manual-control
   mode by itself.
5. **BAPM universal mode.** This is application/power-management state. It is
   useful context, but it does not directly select a fan duty or replace the
   fan pattern.

The automatic controller calculates duty from filtered temperature, target,
controller state, the selected pattern, and hardware-specific calibration.
Changing the target does not directly command a particular duty.

## 2. Units and basic data formats

### Fan duty

Fan duty is a Q10 PWM control value, not RPM:

```text
0       =   0%
256     =  25%
512     =  50%
768     =  75%
1024    = 100%
```

```c
double duty_percent = (double)duty_q10 * 100.0 / 1024.0;
```

The same duty can produce different RPM on different fan models. No reliable
userland tachometer/RPM interface has been identified in this analysis.

### Fixed-point temperatures

The extended-state command contains Q8 values. Convert them with:

```c
double value = (double)raw_q8 / 256.0;
```

The filtered temperature is unsigned Q8. The observed servo-error field is
signed Q8 and behaves approximately like a filtered error relative to target,
but it must not be treated as an instantaneous `temperature - target`
calculation.

### Endianness and alignment

PS5 is little-endian x86-64. Use `memcpy` for fields that may be unaligned:

```c
static uint16_t load_u16(const void *src)
{
    uint16_t value;
    memcpy(&value, src, sizeof(value));
    return value;
}
```

## 3. ioctl encoding

Fan commands use ioctl group `0x8F`, also identified as
`SCE_SYS_FAN_IOCTL_BASE`.

For example, `0xC01C8F07` encodes:

```text
0xC0000000  bidirectional user/kernel transfer
0x001C0000  argument size 0x1c (28 bytes)
0x00008F00  fan command group
0x00000007  command number
```

The encoded size is part of the ABI. A smaller C array is not safe: the kernel
still copies the full encoded number of bytes.

## 4. `/dev/icc_fan` ioctl reference

| ioctl | Size | Direction | Purpose | Confidence |
|---|---:|---|---|---|
| `0xC0168F01` | 22 | in/out | Query or evaluate a fan ID | Partially decoded |
| `0xC0148F02` | 20 | in/out | Read current actual duty | Confirmed |
| `0xC0048F03` | 4 | in/out | Small SET-style command | Purpose unknown |
| `0xC0048F04` | 4 | in/out | Paired GET-style command | Purpose unknown |
| `0xC0068F05` | 6 | in/out | Set manual-duty register | Handler/protocol identified |
| `0xC0068F06` | 6 | in/out | Read manual-duty register | Confirmed |
| `0xC01C8F07` | 28 | in/out | Set runtime autoservo parameters | Confirmed |
| `0xC01C8F08` | 28 | in/out | Get runtime autoservo parameters | Confirmed |
| `0xC0148F09` | 20 | in/out | Read extended state for a channel | Partially decoded |
| `0x80018F0A` | 1 | user to kernel | Select servo pattern 0..3 | Confirmed, write-only |
| `0xC0068F0B` | 6 | in/out | Read actual duty for a selected channel | Confirmed in live testing |

Unknown fan commands normally fail with `ENOTTY`. Pattern values greater than 3
fail with `EINVAL` in the analyzed kernels.

Common definitions:

```c
#define ICC_FAN_DEVICE            "/dev/icc_fan"

#define ICC_FAN_QUERY_ID          0xC0168F01UL
#define ICC_FAN_GET_CURRENT_DUTY  0xC0148F02UL
#define ICC_FAN_SET_SMALL         0xC0048F03UL
#define ICC_FAN_GET_SMALL         0xC0048F04UL
#define ICC_FAN_SET_MANUAL_DUTY   0xC0068F05UL
#define ICC_FAN_GET_MANUAL_DUTY   0xC0068F06UL
#define ICC_FAN_SET_AUTOSERVO     0xC01C8F07UL
#define ICC_FAN_GET_AUTOSERVO     0xC01C8F08UL
#define ICC_FAN_GET_EXTENDED      0xC0148F09UL
#define ICC_FAN_SET_PATTERN       0x80018F0AUL
#define ICC_FAN_GET_CHANNEL_DUTY  0xC0068F0BUL
```

## 5. Command details

### 5.1 Current actual duty: `0xC0148F02`

The 20-byte response begins with:

```text
offset  size  meaning
0x00    2     ICC result
0x02    2     actual duty, Q10
0x04    16    remaining response data
```

```c
static int fan_get_current_duty(int fd, uint16_t *duty_q10)
{
    uint8_t data[20] = {0};

    if (ioctl(fd, ICC_FAN_GET_CURRENT_DUTY, data) < 0)
        return -1;

    *duty_q10 = load_u16(data + 2);
    return 0;
}
```

This is the ioctl used by `sceKernelGetCurrentFanDuty()`. In live testing its
value matched the API, extended-state duty, and channel-duty queries.

### 5.2 Runtime autoservo: `0xC01C8F07` and `0xC01C8F08`

Both commands require exactly a 28-byte user buffer:

```text
offset  size  meaning
0x00    2     ICC result
0x02    1     selector/channel
0x03    1     flag or auxiliary selector
0x04    24    six little-endian uint32 parameters
```

The target temperature in degrees Celsius is byte `buffer[5]`.

For GET, the kernel uses `buffer[2]` as selector and returns the six parameter
words in `buffer[4..27]`. For SET, all 26 bytes in `buffer[2..27]` are passed to
the controller. Therefore a SET buffer containing zeros everywhere except byte
5 will also zero unrelated controller parameters.

Selector 0 is used by existing implementations for the primary/default
profile. The meaning of every selector on every hardware revision is not yet
known.

The safe operation is always:

```text
GET current 28 bytes
    -> copy the returned object
    -> change only byte 5
    -> SET the complete object
    -> GET again and verify
```

Static analysis found the same 28-byte object in every examined implementation;
no smaller SET or GET variant was identified.

### 5.3 Extended state: `0xC0148F09`

The 20-byte object is only partially decoded:

| Offset | Type | Observed meaning |
|---:|---|---|
| `0x04` | `int32_t`, signed Q8 | Filtered servo error relative to target; still partly inferred |
| `0x0c` | `uint32_t` | Current duty Q10; matched `8F02` |
| `0x10` | `uint32_t`, unsigned Q8 | Filtered temperature in °C |

```c
uint8_t ext[20] = {0};

if (ioctl(fd, ICC_FAN_GET_EXTENDED, ext) == 0) {
    int32_t error_q8;
    uint32_t duty_q10;
    uint32_t temperature_q8;

    memcpy(&error_q8, ext + 4, sizeof(error_q8));
    memcpy(&duty_q10, ext + 12, sizeof(duty_q10));
    memcpy(&temperature_q8, ext + 16, sizeof(temperature_q8));

    printf("error=%.3f duty=%u filtered=%.3f C\n",
           error_q8 / 256.0,
           duty_q10,
           temperature_q8 / 256.0);
}
```

The filtered values can lag behind the instantaneous CPU and SoC API readings.

### 5.4 Manual duty: `0xC0068F05` and `0xC0068F06`

The six-byte object is:

```text
offset  size  meaning
0x00    2     ICC result
0x02    1     fan selector/ID
0x03    1     padding or flag
0x04    2     manual duty, Q10
```

The manual-duty register is not the current actual duty. During live tests, the
manual register remained zero while automatic actual duty changed between 195
and 338. The automatic controller may ignore or overwrite a manual value.

Reading `8F06` also produced a kernel-log message on the tested console, so it
should not be polled at high frequency.

Do not build a custom software fan curve around `8F05` until entry into a true
manual mode, thermal fail-safes, and a reliable return to automatic control are
understood.

### 5.5 Servo pattern: `0x80018F0A`

The argument is one byte and must be in the range 0..3:

```c
static int fan_set_pattern(int fd, uint8_t pattern)
{
    if (pattern > 3) {
        errno = EINVAL;
        return -1;
    }
    return ioctl(fd, ICC_FAN_SET_PATTERN, &pattern);
}
```

This command selects a preloaded servo-parameter bank. It does not set a duty,
temperature, or 28-byte autoservo object. The kernel applies the selected value
to four fan channels.

In the analyzed ShellCore launch flow:

```text
NAVIGATION_ACTIVE / home  -> FAN pattern 0
BIG_APP / game            -> FAN pattern 1
```

Patterns 2 and 3 do not yet have reliable user-facing meanings.

No matching hardware getter was found. `8F0A` is write-only, and neither the
autoservo blob nor actual duty uniquely identifies the current pattern.
ShellCore maintains a software cache for its own last successful pattern, but
that cache becomes stale if another process calls `8F0A` directly.

### 5.6 Per-channel duty: `0xC0068F0B`

This command uses the same six-byte envelope as manual duty: selector at offset
2 and returned duty at offset 4. In live testing, selectors 0..3 all returned
the same actual duty while selectors 4..7 failed. Sensor and channel counts must
not be assumed to be identical across hardware revisions.

### 5.7 Fan-ID and small paired commands

`8F01` exposes information associated with a fan ID, but its complete 22-byte
layout is not decoded. On the tested console, IDs 0..3 reported an active state
byte while IDs 4..7 did not.

`8F03` and `8F04` form a small SET/GET-style pair with four-byte ioctl objects.
Their exact purpose is not confirmed and they should not be used for control.

## 6. Temperature and status APIs

Prefer the exported libkernel functions when available:

```c
int sceKernelGetCpuTemperature(int *out_celsius);
int sceKernelGetSocSensorTemperature(int sensor_id, int *out_celsius);
int sceKernelGetCurrentFanDuty(uint16_t *out_duty,
                               uint64_t *out_chassis_info);
```

CPU and SoC temperatures come from `/dev/sbi`, not `/dev/icc_fan`. The
corresponding direct commands are:

| Device | ioctl | Size | Purpose |
|---|---|---:|---|
| `/dev/sbi` | `0x4004A501` | 4 | Read CPU temperature |
| `/dev/sbi` | `0xC008A502` | 8 | Query a SoC sensor by ID |

The number of SoC sensors is hardware-dependent. System software enumerates IDs
from zero until `sceKernelGetSocSensorTemperature()` reports unsupported input.
A compatible monitor should do the same with a defensive upper bound. No SoC
sensors should be treated as a supported CPU/fan-monitoring configuration, not
as a fatal error.

The second output of `sceKernelGetCurrentFanDuty()` is chassis information, not
a second duty value.

## 7. What happens when a game starts

ShellCore's SystemStateMgr stores a fan-pattern field in each system-state
profile. When the power mode changes, ShellCore compares the requested pattern
with its cached pattern and calls the libkernel wrapper for `8F0A` only when it
changes.

The observed sequence is:

```text
ShellCore chooses BIG_APP
    -> ShellCore selects FAN pattern 1 through 8F0A
    -> ICC activates the corresponding preset bank
    -> minimum duty and possibly the active target change
    -> the automatic thermal servo continues from that preset
```

In live testing, entering a game changed duty from `195/1024` to `338/1024`
before the universal-mode transition became visible. In one controlled run, a
custom 85 °C target reverted to the stock 91 °C target at the same pattern
change. ShellCore did not call `8F07`; the pattern switch caused ICC to activate
the stock parameters for the selected bank.

Other hardware showed a game floor of `190/1024` rather than 338. A pattern
number therefore does not imply one universal duty. Board/vendor calibration,
chassis revision, load, ambient temperature, and controller history all matter.

## 8. Safe target-temperature update

The minimal portable implementation is:

```c
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int get_autoservo(int fd, uint8_t selector, uint8_t cfg[28])
{
    memset(cfg, 0, 28);
    cfg[2] = selector;
    return ioctl(fd, ICC_FAN_GET_AUTOSERVO, cfg);
}

static int set_target_temperature(int fd, uint8_t selector,
                                  uint8_t target_c)
{
    uint8_t current[28];
    uint8_t verify[28];

    if (get_autoservo(fd, selector, current) < 0)
        return -1;

    current[5] = target_c;
    if (ioctl(fd, ICC_FAN_SET_AUTOSERVO, current) < 0)
        return -1;

    if (get_autoservo(fd, selector, verify) < 0)
        return -1;

    if (verify[5] != target_c) {
        errno = EIO;
        return -1;
    }

    return 0;
}

int apply_target(uint8_t target_c)
{
    int fd = open(ICC_FAN_DEVICE, O_RDWR);
    if (fd < 0)
        return -1;

    int result = set_target_temperature(fd, 0, target_c);
    int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return result;
}
```

Important rules:

1. Test `open()` with `fd >= 0`; descriptor 0 is valid.
2. Always use a 28-byte buffer for `8F07` and `8F08`.
3. Always GET before SET. Never construct a production SET from a zero-filled
   buffer with only byte 5 assigned.
4. Change only the intended target byte and preserve all other returned bytes.
5. Perform an immediate GET readback. For a controlled test, compare all 28
   bytes with the intended object, not only the target.
6. Validate target policy in userland. The analyzed kernel forwards the target
   byte without enforcing a safe temperature range.
7. Keep automatic thermal control enabled. A lower target requests more
   aggressive cooling but does not guarantee the chip will remain at exactly
   that temperature.
8. Monitor CPU temperature, available SoC sensors, filtered temperature, actual
   duty, and the complete autoservo object after a change.
9. Treat a system pattern transition as a new configuration epoch. Re-read the
   current 28-byte object; never replay a stale object over a newly selected
   system profile.
10. On repeated device, sensor, SET, or readback failures, stop writing and leave
    the stock controller in charge.

For bounded experiments, retain the complete baseline and restore it only while
the same system profile is still active and the current object still matches
the object written by the test. Restoring an old home or game object after the
system has changed patterns can overwrite valid system-owned parameters.

A persistent target guard can poll `8F08` and modify byte 5 in a freshly read
object after a system transition. This is an explicit policy override. It must
never reuse a stale 28-byte blob or force manual duty as a substitute for the
stock servo.

## 9. Monitoring recommendations

A useful monitor records:

- CPU temperature;
- every dynamically discovered SoC temperature, or at least the maximum;
- current actual duty from the API or `8F02`;
- the complete `8F08` object and target byte 5;
- filtered temperature, controller error, and duty from `8F09`;
- per-channel duty from `8F0B` when needed;
- chassis information;
- ShellCore power-mode events and fan pattern when observable;
- universal mode as supporting context only.

Avoid polling `8F06` continuously because it emitted a kernel-log line in live
tests. Do not infer RPM from duty, pattern from the autoservo object, or a game
profile from universal mode alone.

## 10. Can the full fan curve be edited?

No arbitrary multi-point fan-curve API has been confirmed in userland.

The available interfaces can:

- change the runtime target temperature;
- read or write a manual-duty register;
- select one of four preloaded servo patterns;
- read several controller-state objects.

The complete calibrated curve appears to live in the approximately 1710-byte
board/vendor-specific table loaded by the kernel. Editing or replacing that
table requires the full format and hardware-selection rules to be understood.
Copying it between PS5 revisions is unsafe.

A userland loop that maps temperature to manual duty would replace important
stock-controller feedback and fail-safe behavior. It should not be treated as a
safe curve editor without a confirmed manual mode, sensor-failure handling,
duty limits, watchdog behavior, and a proven return to automatic control.

For practical cooling changes, preserving the factory calibration and changing
only the autoservo target is the least invasive known method.

## 11. Compatibility and access

- Access to `/dev/icc_fan` and `/dev/sbi` depends on authid, sandbox policy, and
  kernel patches. Failure to open the device does not prove that the device is
  absent.
- Do not hard-code a stock target of 91 °C. Most analyzed tables used 91, but
  one board-specific table used 69. Always read the active value first.
- Do not hard-code the number of SoC sensors or assume that all fan/channel IDs
  exist.
- The `8F07/08` size and byte-5 target were stable across all analyzed
  implementations and live tests, but every other field should remain opaque
  unless separately verified.
- Different chassis and fan assemblies can use different minimum duties and
  calibration data.

## 12. Remaining unknowns

- exact semantics of servo patterns 2 and 3;
- full layouts of `8F01`, `8F03`, `8F04`, and the unknown parts of `8F09`;
- the complete set of valid fan selectors on every PS5 revision;
- an accessible real-RPM/tachometer interface;
- the format of the approximately 1710-byte calibration table;
- which autoservo fields other than target byte 5 are safe to edit;
- a reliable hardware readback of the currently selected servo pattern.
