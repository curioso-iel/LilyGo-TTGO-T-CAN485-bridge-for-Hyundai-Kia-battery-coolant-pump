# LilyGo Pump Control **PB1**

**A dedicated RS485-to-CAN bridge for controlling an original Hyundai/Kia battery coolant pump with a LilyGo T-CAN485.**

This project receives commands using the custom **PB1** protocol and generates CAN frames for the **375W5-K4000 OS EV BAT** rear battery coolant pump. It also returns pump feedback to the controller over RS485.

It was developed alongside the touchscreen port in the [curioso-iel/Battery-Emulator fork](https://github.com/curioso-iel/Battery-Emulator), using a Waveshare ESP32-S3-Touch-LCD-5B as the user interface and PB1 controller.

> **Experimental hardware-control software**
>
> This is not a certified safety controller or emergency-stop system. A zero command, successful UART write or successful CAN enqueue does not guarantee that the pump has physically stopped.
>
> **The tested pump can run in an OEM fallback mode when CAN commands disappear.** Disconnecting CAN, switching off the LilyGo or resetting it may therefore make the pump run rather than stop. Keep this distinction in mind when designing and testing the cooling circuit.

## Contents

- [Purpose and architecture](#purpose-and-architecture)
- [Hardware compatibility](#hardware-compatibility)
- [Features](#features)
- [Wiring](#wiring)
- [Build and upload](#build-and-upload)
- [Operation and failure handling](#operation-and-failure-handling)
- [PB1 protocol specification](#pb1-protocol-specification)
- [Pump CAN messages](#pump-can-messages)
- [Serial diagnostics](#serial-diagnostics)
- [Commissioning sequence](#commissioning-sequence)
- [Troubleshooting](#troubleshooting)
- [Validation status and limitations](#validation-status-and-limitations)
- [Repository files](#repository-files)
- [Credits and licensing](#credits-and-licensing)

## Purpose and architecture

The LilyGo is a **separate pump controller**, not the main Battery-Emulator device:

```text
Battery and inverter
        |
   Battery/inverter CAN
        |
   Waveshare LCD-5B
   Battery-Emulator + PB1 controller
        |
   Dedicated RS485 pair
        |
   LilyGo T-CAN485
   This firmware
        |
   Separate pump-only CAN
        |
   375W5-K4000 battery coolant pump
```

**Do not connect the LilyGo pump CAN to the battery/inverter CAN bus.** The OEM frames generated here are intended only for the pump's dedicated segment.

The receiver can be used with another PB1 controller that implements the packet format, handshake and timing described below. It is **not a Modbus RTU device**, a generic CAN gateway or a USB-commanded manual controller.

## Hardware compatibility

| Component | Intended hardware |
|---|---|
| Receiver board | Original **LilyGo T-CAN485**, ESP32 |
| CAN peripheral | ESP32 TWAI, normal mode, 500 kbit/s |
| RS485 peripheral | Onboard MAX13487 automatic-direction circuit |
| Pump tested | **375W5-K4000 OS EV BAT**, rear battery coolant pump |
| Companion controller | Waveshare **ESP32-S3-Touch-LCD-5B**, with the fork's PB1 module |

The **LilyGo T-2CAN is a different board**. Do not assume this firmware's pins or build profile are suitable for it.

The front **36910-0E650 OS/DE EV PE** pump uses different CAN messages. This project does not provide a selectable implementation for that pump, nor universal compatibility with Hyundai/Kia pumps.

### Pin configuration

| Function | ESP32 GPIO / setting |
|---|---|
| Peripheral 5 V rail enable | GPIO16, HIGH |
| CAN TX | GPIO27 |
| CAN RX | GPIO26 |
| CAN transceiver mode | GPIO23, LOW |
| RS485 UART TX | GPIO22 |
| RS485 UART RX | GPIO21 |
| RS485 `/RE` control | GPIO17, HIGH, following the board's automatic-mode example |
| RS485 `/SHDN` | GPIO19, HIGH |
| UART instance | `Serial2` |
| USB serial logging | 115200 baud |

These assignments follow the T-CAN485 configuration used during development and the board examples. Some published tables have conflicting enable-pin labels; do not substitute GPIO9 for GPIO19 based on an unrelated table or board revision without checking the actual hardware.

## Features

- Dedicated binary PB1 request/reply communication over RS485.
- Zero command at startup; no stored manual speed to restore.
- Nonzero output gated by a zero handshake and healthy pump feedback.
- Command ceiling **153 decimal / `0x99`**.
- CAN demand and purge-off messages on a normal 100 ms cadence.
- **750 ms local watchdog** for accepted PB1 requests.
- CRC, session, sequence, range and reserved-field validation.
- Rejection of duplicate and out-of-order requests without refreshing the watchdog.
- Feedback reporting for current, the OEM internal-speed byte and alarm bytes.
- Nonblocking **20 ms reply guard**, with readiness recalculated when replying.
- Bounded processing of UART/CAN queues and detection of long loop gaps.
- CAN bus-off recovery attempts without restoring an old nonzero demand.
- Periodic serial diagnostics, without per-frame logging.

No Wi-Fi, web interface, OTA implementation, SOC logic, battery/inverter emulation or persistent application settings are implemented in this receiver.

## Wiring

Make wiring changes with the relevant supplies switched off. Use suitable wiring, protection and a power supply rated for the actual pump. Do not power the pump from the LilyGo's USB connection or a GPIO/5 V output.

### RS485: controller to LilyGo

For the development setup:

| Controller | LilyGo T-CAN485 |
|---|---|
| RS485 A | RS485 A |
| RS485 B | RS485 B |
| Communication reference ground | Appropriate low-voltage communication ground |

Use a twisted pair for A/B. RS485 is differential but still has common-mode voltage limits; plan grounding and isolation for the installed system. Two non-isolated USB connections to the same PC may already provide a common reference during bench tests, but that is not a grounding design for a permanent installation.

A/B naming is not consistent across all vendors. In this particular setup, A-to-A and B-to-B were verified by valid captured requests and replies. Do not confuse swapping A/B with changing UART RX/TX GPIO assignments.

Keep this RS485 segment dedicated to PB1, with one controller. Do not add a Modbus master or another device that transmits unsolicited traffic.

### CAN: LilyGo to pump

Connect LilyGo CAN H and CAN L to the verified pump CAN H and CAN L respectively, with an appropriate reference ground and separate pump power.

The pump connector's physical pinout must be verified for the exact pump and harness. This README intentionally does not provide a universal wire-colour assignment: colours alone are not evidence of pin function.

### 120-ohm termination

CAN and RS485 are **separate buses**, each with its own termination requirements. Where required by the cable/topology, use approximately **120 ohms at each end of the segment**, accounting for resistors already fitted to the boards or connected equipment.

Do not add a third terminator just because an Ethernet tap or USB analyser is connected. Do not alter the battery/inverter CAN termination when diagnosing this pump's RS485 link.

The Waveshare's RS485 termination was changed during troubleshooting; removing it alone did not fix reception. The later working RX configuration was a software change on the Waveshare. Final termination should be checked for the actual cable lengths and installed topology rather than copied blindly from an intermediate bench test.

## Build and upload

### Project layout

```text
LilyGo-Pump-PB1/
|-- platformio.ini
|-- src/
|   |-- main.cpp
|   `-- pb1.h
|-- README.md
|-- .gitignore
`-- LICENSE
```

### Tool versions

The supplied configuration is:

```ini
[platformio]
default_envs = lilygo_pump_pb1

[env:lilygo_pump_pb1]
platform = espressif32@6.9.0
board = esp32dev
framework = arduino
monitor_speed = 115200
monitor_filters = time, log2file
build_flags = -Wall -Wextra
```

Use the pinned platform for the documented revision. It uses the Arduino-ESP32 2.x / ESP-IDF 4.x API, including the task-watchdog initialization signature. Do not substitute the Waveshare's pioarduino platform or ESP32-S3 board profile without adapting and testing the firmware.

### Compile

Extract or clone the repository, open its root in PlatformIO, then run:

```sh
pio run -e lilygo_pump_pb1
```

For Windows PowerShell installations where `pio` is not in PATH, the development machine used:

```powershell
& "$env:USERPROFILE/.platformio/penv/Scripts/platformio.exe" run -e lilygo_pump_pb1
```

Use your actual PlatformIO executable path if different. Run from the directory containing this project's `platformio.ini`.

### Upload

**Uploading this firmware replaces the application currently running on the LilyGo.** Preserve the previous project/image if the board was previously acting as a Battery-Emulator. Do not repurpose an active battery/inverter controller without first putting that installation into an appropriate test/stopped state.

Identify the LilyGo port using `pio device list`. Example for a LilyGo currently on COM5:

```sh
pio run -e lilygo_pump_pb1 -t upload --upload-port COM5
```

Example using the full PlatformIO executable in PowerShell:

```powershell
& "$env:USERPROFILE/.platformio/penv/Scripts/platformio.exe" run -e lilygo_pump_pb1 -t upload --upload-port COM5
```

COM5 was the LilyGo and COM4 the Waveshare during development; these assignments can change. Check the actual board before uploading. Close any serial monitor holding the port first. No full-flash erase is required by these instructions.

### Monitor

```sh
pio device monitor -p COM5 -b 115200
```

Or in Windows PowerShell:

```powershell
& "$env:USERPROFILE/.platformio/penv/Scripts/platformio.exe" device monitor -p COM5 -b 115200
```

The startup banner should identify:

```text
PDS193 T-CAN485 PB1 19200 8N1; reply guard 20ms; init=...; boot ZERO
```

The `init` result refers to CAN driver initialization/startup. Normal `PB1` and `PB1TX` diagnostics continue afterwards when serial output is available.

## Operation and failure handling

### Startup and zero handshake

The receiver starts with a zero demand and, while CAN is operational, attempts to send zero/purge-off messages every 100 ms. It can reply to PB1 requests even if the pump is absent, so the controller can distinguish RS485 connectivity from pump readiness.

A new session must start with a **zero request**. A new-session nonzero request is rejected. Once a zero request is accepted and local feedback checks are healthy, the receiver can report readiness and subsequently accept a nonzero demand.

A successfully accepted PB1 request is not necessarily a command applied to the pump: a disarmed receiver can accept an otherwise valid, sequential nonzero request while keeping local output zero. Its response remains not-ready, prompting the companion controller to return to a zero handshake.

### Local pump-health checks

Nonzero output requires all of the following:

- CAN controller reported running.
- No active local TX-fault latch.
- A received **standard, non-RTR `0x4E4` frame with DLC 8**.
- Feedback no older than **500 ms**, measured from software reception.
- Feedback byte 0 bit 1 clear.
- Feedback byte 7 equal to zero.
- Receiver armed and an accepted PB1 request younger than 750 ms.

The 500 ms age is not a timestamp generated by the pump. Queue backlogs can affect reception age; this implementation drops excessive backlogs rather than claiming complete freshness assurance.

### Watchdog and rearming

At **750 ms or more without an accepted request**, the receiver clears its demand and disarms. If CAN is running and the previous queued demand was nonzero, it attempts zero without waiting for the next normal 100 ms period.

Malformed requests and rejected duplicates/out-of-order frames do not renew that timer. Sending a reply also does not renew it. Recovery requires another accepted zero request and healthy feedback before a new nonzero demand can be used.

A loop interval above **200 ms** also disarms the receiver. The ESP32 task watchdog is configured for approximately **3 seconds**. These are software measures, not hard real-time guarantees or independent power-cut mechanisms.

### CAN faults

Frames use single-shot transmission rather than indefinitely retrying one command. Enqueue failures or reported TX-failure alerts latch a fault and disarm the receiver. Queued frames may be cleared, but a frame already transmitting cannot be recalled.

Bus-off recovery/restart is attempted at spaced intervals, without restoring the previous nonzero demand. TX readiness can recover after a TX-success alert with the locally tracked zero demand and fresh feedback; this is not a per-frame physical-action acknowledgment from the pump.

### Reply turnaround

After a request, the receiver waits at least **20 ms from acceptance and from the last received UART byte**, with no unread UART bytes pending, before replying. This is implemented without a blocking `delay(20)` or `Serial.flush()` in the reply path.

CAN service and watchdog checks continue during the guard interval. Readiness is recalculated at response time. A pending response older than 100 ms is discarded; excessive UART backlog or a long loop gap can also cancel it.

The 20 ms guard was retained from a diagnostic trial. It was **not** the change that alone resolved the Waveshare's zero-byte RX problem.

### Temperature automation belongs to the controller

The LilyGo does **not** calculate temperature hysteresis. PB1 carries temperature metadata, but this receiver applies the controller's demand subject to its local checks.

In the companion Waveshare implementation, automatic cooling starts above 30 C and its latch clears at or below 25 C. Automatic demand can override manual zero. Disabling/closing a UI menu is not equivalent to stopping automatic operation.

The controller's temperature freshness and thermal policy therefore need their own validation. This receiver cannot independently protect a battery against overheating when the controller sends zero or loses communication.

## PB1 protocol specification

### Serial and transaction parameters

| Setting | Value |
|---|---|
| Baud rate | 19200 |
| Format | 8N1, no parity |
| UART | Serial2 on GPIO21 RX / GPIO22 TX |
| Packet length | 20 bytes |
| Normal controller poll period | 250 ms in the companion implementation |
| Companion response deadline | 200 ms |
| Receiver request watchdog | 750 ms |
| Partial-frame inactivity reset | Greater than 50 ms between received bytes |
| Response guard | At least 20 ms |
| Pending response expiry | Greater than 100 ms |

All multibyte fields below are **little-endian**. No carriage return, newline or text wrapper is appended to a PB1 frame.

### Request layout

| Bytes | Meaning |
|---|---|
| 0-1 | Header: `A5 5A` |
| 2 | Version: `01` |
| 3 | Request type: `01` |
| 4-7 | Nonzero 32-bit session nonce |
| 8-9 | Nonzero 16-bit sequence |
| 10 | Requested raw demand, **0-153** |
| 11 | Metadata flags: bit 0 temperature valid; bit 1 automatic request; bit 2 manual setting nonzero |
| 12-13 | Maximum battery temperature in tenths of a degree C, signed 16-bit; `INT16_MIN` when invalid |
| 14-17 | Reserved: all zero |
| 18-19 | CRC16/MODBUS of bytes 0-17 |

Validation requires a valid header/version/type/CRC, nonzero nonce and sequence, demand no greater than 153, no flag bits above bit 2, and zero reserved bytes. If temperature-valid is set, temperature must be between -400 and 1000 tenths of a degree C; otherwise the invalid sentinel is required.

The current receiver checks that metadata encoding, but does not enforce the controller's thermal policy or infer a speed from the flags.

### Reply layout

| Bytes | Meaning |
|---|---|
| 0-1 | Header: `A5 5A` |
| 2 | Version: `01` |
| 3 | Reply type: `02` |
| 4-7 | Echoed session nonce |
| 8-9 | Echoed sequence |
| 10 | **Echoed request demand**, not measured speed or locally applied demand |
| 11 | Status flags described below |
| 12-13 | Feedback age in milliseconds, saturated at 65535; 65535 when never seen |
| 14 | Pump feedback byte 0 |
| 15 | Pump feedback byte 1 |
| 16 | Pump feedback byte 5 |
| 17 | Pump feedback byte 7 |
| 18-19 | CRC16/MODBUS of bytes 0-17 |

Reply flags:

- **Bit 0:** CAN/TX readiness argument is true, receiver armed and request link current. The main loop also reevaluates pump health before replying.
- **Bit 1:** pump feedback considered fresh.
- **Bit 2:** an alarm is present in the last received pump feedback.
- **Bits 3-7:** zero.

Consumers must validate flags, feedback age and alarm bytes together. Feedback bytes may retain the last received values even when no longer fresh. A `0x03` status is the expected healthy/ready combination in the tested handshake, but it is not a physical stop/speed certificate.

### CRC

```text
CRC-16/MODBUS
Initial value: 0xFFFF
Reflected polynomial: 0xA001
Input: first 18 bytes
Output: low byte, then high byte
Reference: "123456789" -> 0x4B37
```

### Session and sequence behavior

The controller must change the nonce before wrapping the 16-bit sequence. Within a session, sequences must strictly increase; gaps are allowed, repeats and earlier values are rejected.

A new nonce requires a zero request. The last eight retired nonces are retained in RAM to reject recently retired sessions. This history is finite and disappears on reboot.

**PB1 is not authenticated.** CRC detects corruption, not a malicious controller. It has no cryptographic authentication, encryption or sender timestamps and does not provide permanent replay protection. Keep the segment dedicated and physically controlled.

## Pump CAN messages

All commands are standard 11-bit CAN frames, DLC 8, at 500 kbit/s:

| CAN ID | Direction | Payload / use |
|---|---|---|
| `0x4DE` | LilyGo to pump | Byte 0 = raw demand; bytes 1-7 = zero |
| `0x523` | LilyGo to pump | All eight bytes zero; purge OFF |
| `0x4E4` | Pump to LilyGo | Feedback used for readiness, current and alarms |

The normal transmit period is 100 ms for each command. Purge activation is not implemented.

The feedback interpretation comes from community reverse engineering and bench observations, not a manufacturer-certified specification:

- Byte 0 bit 1 is treated as an alarm.
- Byte 1 is an OEM internal speed/set-speed indication, not calibrated RPM.
- Byte 5 is interpreted as current in tenths of an ampere.
- Any nonzero byte 7 is treated as an alarm by this implementation.

### Raw command versus displayed percentage

The receiver accepts **0 through 153**. The companion UI presents that chosen range as:

```text
0%   -> 0x00 (0)
100% -> 0x99 (153)
```

100% does not mean the maximum possible OEM command or calibrated maximum rotation. In the companion UI, the retained raw increments are 0, 5, 10, ... 150, 153, resulting in approximately 3-4 percentage-point display steps.

Separate bench observations included approximately 4.0-4.1 A of reported current at raw `0x99`; a higher `0xFF` command was tested separately before selecting the lower ceiling. This firmware rejects commands above `0x99`. Those observations are not a calibration table or a guarantee of current under other hydraulic conditions.

## Serial diagnostics

Illustrative lines:

```text
PB1 link=1 armed=1 CAN=1 fresh=1 want=0 queued=0 B1=0 A=0.0 alarm=00/00 RX=100 ok=40 reject=0 crc=0 timeout=0 txerr=0 fail=0 replyerr=0
PB1TX guard=20ms queued=40 dropped=0 errors=0 pending=0
```

These examples illustrate the format, not live measurements.

| Field | Meaning |
|---|---|
| `link` | A request was accepted less than 750 ms ago |
| `armed` | Receiver has completed a zero-handshake state and has not been disarmed |
| `CAN` | Controller reported running; not by itself complete pump health |
| `fresh` | Feedback received within 500 ms |
| `want` | Internal requested raw demand, cleared on disarm |
| `queued` in `PB1` | Last tracked demand whose command pair was queued successfully; can also be reset to zero on a queue fault |
| `B1` | Last pump feedback byte 1 |
| `A` | Byte 5 / 10 when fresh; **-1.0** when not fresh |
| `alarm` | Last feedback bytes 0 and 7, in hexadecimal |
| `RX` | Count of accepted-format pump feedback frames |
| `ok` | Accepted PB1 requests |
| `reject` | Receiver-level rejected requests, including session/sequence rejection |
| `crc` | Invalid parser windows; includes malformed data, not only CRC failures |
| `timeout` | Count of request-watchdog expiry episodes |
| `txerr` | CAN enqueue-pair failures |
| `fail` | Observed CAN TX-failure alert events, not necessarily an exact count of every failed frame |
| `replyerr` | RS485 response write/buffer failures |
| `queued` in `PB1TX` | Responses accepted by the UART, not acknowledgments from the controller |
| `dropped` | Pending replies discarded or superseded |
| `pending` | A response is currently waiting for its guard interval |

The two lines are emitted at different times. Their counters can differ by one without indicating an error. A sampled `pending=1` is not a stuck response if `PB1TX queued` continues increasing.

Output is attempted about once per second, with the reply diagnostic offset from the main line. Lines can be omitted if the serial buffer is busy or the formatted message does not fit its fixed buffer. No application command parser is attached to the USB console.

## Commissioning sequence

1. **Verify hardware and source revision.** Confirm original T-CAN485 and the exact rear battery pump. Preserve any previous firmware project.
2. **Compile and upload.** Check for SUCCESS and the PDS193 banner on the LilyGo's actual port.
3. **Establish the pump-only CAN segment.** Use verified pump power/wiring and termination. Confirm increasing `RX`, `CAN=1` and fresh feedback. Do not proceed to nonzero commands while faults/alarms remain.
4. **Connect the dedicated PB1 controller.** Configure 19200 8N1. Confirm `ok` and `PB1TX queued` increasing; the Waveshare sender must also receive and accept replies.
5. **Verify the zero handshake.** With automatic demand inactive, inspect the actual pump response and current. Do not use `queued=0` alone as proof of stopping.
6. **Try a small manual demand.** Observe command, feedback and current together, with the pump in an appropriate supplied/primed hydraulic circuit. Do not assume a percentage is an RPM measurement.
7. **Return to manual zero.** Confirm the physical response, remembering that controller-side AUTO can override manual zero.
8. **Test loss of RS485 only.** Keep LilyGo and pump CAN/power intact; verify watchdog behavior and the actual pump response. Restore the link and check zero rearming. Do not substitute a CAN-disconnect test for this test.
9. **Validate prolonged operation and thermal behavior separately.** Record firmware versions, topology, supply conditions and logs. Test results for one configuration do not certify another.

## Troubleshooting

### No PB1 requests accepted

Check baud/format, controller firmware, A/B/reference wiring, supply and termination. PB1 must be sent as binary transparent data, not translated through Modbus TCP/RTU. Check the controller is actually transmitting; local UART queue success alone is not electrical evidence.

### LilyGo reports ready, but Waveshare controls remain grey

Compare both ends. The LilyGo can receive requests and enqueue replies while the Waveshare receives nothing. During development, an independent Ethernet capture confirmed valid replies on the bus.

The full Waveshare integration was fixed by its **PDS199 RX configuration**, after Arduino UART initialization and pin reservation:

```cpp
gpio_set_direction(GPIO_NUM_43, GPIO_MODE_INPUT);
uart_set_pin(UART_NUM_2, 44, 43, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
Serial2.setRxFIFOFull(1);
Serial2.setRxTimeout(2);
```

**These GPIO43/44 changes belong to the Waveshare, not this LilyGo firmware.** The tested sequence solved that setup; its individual decisive operation was not separately established. The receiver's 20 ms reply guard or removal of termination did not fix the issue alone.

### Request link is current, but no nonzero output

Check fresh pump feedback, alarm bytes, TX-fault state and rearming. `link=1` and `CAN=1` alone are insufficient. After disarming, send zero before a new nonzero request. On the companion controller, invalid temperature disables automatic demand but does not inherently prevent healthy manual operation.

### The pump runs when the LilyGo is disconnected

This matches the fallback behavior observed during development. Restore the intended controller/CAN operation and review the physical cooling-system design. Do not interpret it as proof that the watchdog failed: the watchdog can request zero only while software and the required communication path are operating.

### Errors increase or replies stop

Inspect `crc`, `reject`, `dropped`, `txerr`, `fail` and `timeout`, and compare actual bus traffic where possible. Do not relax packet checks, raise the command ceiling or repeatedly increase reply delays to hide an unexplained failure.

### PlatformIO package problems

Keep this original-ESP32 project separate from the ESP32-S3 Waveshare project and retain its pinned platform. Missing package manifests or tools are installation issues, not evidence of a pump-protocol failure. Avoid deleting the entire PlatformIO installation as a first response.

## Validation status and limitations

During development:

- The receiver was compiled and uploaded on the user's T-CAN485.
- Logs showed ongoing pump feedback and accepted PB1 requests, with queued responses.
- An Ethernet tap captured **80 valid request/reply pairs**, with matching CRC, session, sequence and demand echo.
- Reception in the full Waveshare integration was reported working after its PDS199 configuration fix.
- Separate CAN bench tests demonstrated zero and nonzero pump responses and informed the selected `0x99` ceiling.

Host tests exercised the protocol header and the firmware loop with simulated Arduino/TWAI peripherals: CRC reference and bit mutations, duplicates, sequence checks, zero handshake/rearm, 750 ms boundaries, clock rollover, response layout, malformed streams, 60,000 sequential requests, reply guard, stale pending replies, CAN enqueue failures and selected recovery paths.

**Host tests do not validate electrical timing, hydraulic performance or physical stopping.** Exhaustive end-to-end fault injection, sustained operation, independent current calibration, controller temperature freshness and production safety assessment remain open work.

Additional limitations:

- The request watchdog is evaluated by software, not an independent hardware power cut.
- The receiver cannot guarantee delivery of a zero frame during bus-off or power failure.
- Feedback age is based on reception time and is not a pump-generated sample timestamp.
- Nonce history is finite, in RAM and unauthenticated.
- Some driver/watchdog API results and recovery corner cases need further hardening.
- Logging and scheduling remain timing influences, even with bounded work and output-space checks.
- A source ZIP containing a binary does not establish that the binary exactly matches the source or the running device; record commit/build provenance before publishing firmware releases.

## Repository files

| File | Role |
|---|---|
| [`platformio.ini`](platformio.ini) | Dedicated pinned build environment |
| [`src/main.cpp`](src/main.cpp) | Hardware initialization, CAN/RS485 servicing, reply guard and diagnostics |
| [`src/pb1.h`](src/pb1.h) | Packet helpers, CRC, parser, session handling, watchdog and reply encoding |
| [`.gitignore`](.gitignore) | Excludes generated builds, editor settings and logs |
| [`LICENSE`](LICENSE) | Repository licensing terms |

PDS192 and PDS193 documents may be retained as development history. The README describes the current receiver behavior; the historical term â€œtrialâ€ in source comments records the origin of the reply guard, not a claim that the guard alone fixed the system.

## Credits and licensing

- [Dala's Battery-Emulator](https://github.com/dalathegreat/Battery-Emulator) and its contributors, for the companion system and hardware integration context.
- [The companion Waveshare fork](https://github.com/casaantoes-eng/Battery-Emulator), including its [LCD-5B guide](https://github.com/casaantoes-eng/Battery-Emulator/blob/main/readme_Waveshare_touch_lcd_5b.md).
- [LilyGo T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485), its board documentation and [RS485 example](https://github.com/Xinyuan-LilyGO/T-CAN485/blob/main/example/Arduino/RS485/RS485.ino).
- [OpenInverter: Hyundai Kona EV Coolant Pumps](https://openinverter.org/wiki/Hyundai_Kona_EV_Coolant_Pumps), for community pump protocol information used alongside bench tests.
- [Espressif Arduino-ESP32](https://github.com/espressif/arduino-esp32) and [ESP-IDF](https://github.com/espressif/esp-idf), for UART, TWAI and runtime support.
- [PlatformIO Espressif32 platform](https://github.com/platformio/platform-espressif32), for the build environment.

Development and hardware tests were carried out by the project owner with AI-assisted code and documentation work. That assistance is not independent verification of the implementation or its safety.

See [LICENSE](LICENSE) for the repository's applicable terms. Preserve the notices and licenses of any reused code and third-party dependencies; the repository license does not replace those obligations.

Contributions are welcome: include the exact board/pump revision, firmware commit, reproducible steps and logs, and distinguish proposed behavior, host tests and physical observations. Keep changes to the pump bus separate from battery/inverter control, and avoid presenting unverified fault behavior as a guarantee.
