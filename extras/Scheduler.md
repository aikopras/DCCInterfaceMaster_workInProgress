# DCC Packet Scheduler Library

This library provides a **DCC packet scheduler** for model railway command stations. It prepares locomotive, accessory, and programming packets and sends them to the underlying DCC waveform generator. The hardware-specific waveform generation is explained elsewhere; this document focuses on what the library does from the application point of view.

## 1. General

The library is built around the `DCCPacketScheduler` class. Its main task is to take user commands such as speed changes, function changes, accessory commands, and CV programming requests, and place the corresponding DCC packets on the rails in the correct order.

The scheduler maintains several internal queues so that urgent packets are sent quickly, and packets such as locomotive speed and function packets are refreshed over time.

From the point of view of the rails, the result is:

- a continuous NMRA-style DCC signal,
- normal idle packets when no user command is pending,
- reset packets and programming packets during Service Mode,
- optional RailCom cutout generation,
- optional control of rail power on/off,
- optional broadcast emergency stop.

The library supports:

- locomotive speed control in 14, 28, or 128 speed-step mode,
- locomotive functions from **F0 up to F68** directly,
- binary state control commands for decoder states beyond the standard F0–F28 function groups,
- basic accessory commands,
- extended accessory commands,
- Service Mode CV read, verify, and write,
- PoM (Programming on the Main) write, bit write, and read requests,
- storage and recall of locomotive and accessory state,
- callback hooks for application integration.

The user must create an instance of the `DCCPacketScheduler` class. The `update()` function must be called continuously (e.g. in the Arduino `loop()`), since it is responsible for sending queued DCC packets to the rails.

---

## 2. Setup / initialization / start / stop commands

### Creating the scheduler

Create an instance of `DCCPacketScheduler` in your sketch or application.

### `setup(pin, pin2, steps, format, power)`

This is the main initialization call.

It performs the following actions:

- loads configuration values from EEPROM or equivalent storage,
- assigns the main DCC output pin (`pin`),
- assigns the secondary output pin (`pin2`),
- initializes the waveform generator,
- sets the initial rail power state,
- stores the default speed-step mode,
- stores the accessory addressing format,
- starts the command station by sending reset packets followed by idle packets.

**Note:**  
The exact function of `pin2` depends on the hardware mode. The choice between these modes is made by including or removing the `#define Z21PG` at the top of the `DCCHardware.cpp` file.

- In *Z21PG mode* (default), `pin2` outputs an **inverted DCC signal**.
- In *HQ mode*, `pin2` is used as a **Enable / RailCom cutout signal** and does not carry a DCC waveform.


Parameters:

- `pin`: main DCC output pin (always carries the DCC waveform)
- `pin2`: secondary output pin  
  - *Z21PG mode*: inverted DCC signal  
  - *HQ mode*: RailCom cutout control signal
- `steps`: default locomotive speed step mode\
  Use one of:
  - `DCC14`
  - `DCC28`
  - `DCC128`
- `format`: accessory addressing format
  Use one of:
  - `ROCO`
  - `IB`
- `power`: initial rail power state
  Use one of:
  - `ON`
  - `OFF`
  - `ESTOP`
  - `SHORT`
  - `SERVICE`

### `update()`

This function must be called repeatedly from `loop()`.

It is the heart of the scheduler. It:

- checks whether the waveform generator can accept a new packet,
- selects the next packet from the internal queues,
- handles Service Mode programming sequences,
- maintains packet refresh,
- sends the next DCC packet to the hardware layer.

Without repeated calls to `update()`, queued commands will not reach the rails.

### `setpower(state, notify = false)`

Controls the rail output state.

Supported states:

- `ON`: normal DCC signal on the rails
- `OFF`: no rail power output
- `ESTOP`: emergency stop state
- `SHORT`: rail output disabled because of short circuit condition
- `SERVICE`: Service Mode / programming track mode

When the state changes to `OFF` or `SHORT`, the output is stopped. In other states, the output is enabled.

If `notify` is `true`, the weak callback `notifyRailpower()` is called when available.

### `getpower()`

Returns the current rail power state.

### `setrailcom(bool rc = true)`

Enables or disables RailCom cutout generation in the DCC output.

### `getrailcom()`

Returns whether RailCom cutout generation on the DCC output is enabled.

### `getRailComStatus()`

Returns the actual current RailCom cutout status on the rails. This is useful if external hardware needs to know whether a cutout is currently in progress.

### `eStop()`

Sends a broadcast emergency stop packet and switches the rail power state to `ESTOP`.

This is intended for stopping all decoders immediately while still keeping the rails powered.

### `enable_additional_DCC_output(pin)`

***Z21PG mode only:*** Enables an additional DCC output signal on a third pin. This extra output is intended for uses such as S88 or LocoNet interfaces and does **not** include normal rail power shutdown behavior or RailCom cutout behavior.

### `disable_additional_DCC_output()`

***Z21PG mode only:*** Disables the additional DCC output.

---

## 3. Locomotive-specific commands

The library stores locomotive state internally in slots. For each active locomotive it keeps:

- address,
- speed-step mode,
- speed and direction,
- function groups.

This allows the application to query the last known state.

### Speed commands

#### `setSpeed(address, speed)`

Sets speed using the default speed-step mode selected in `setup()`.

#### `setSpeed14(address, speed)`
#### `setSpeed28(address, speed)`
#### `setSpeed128(address, speed)`

Set speed explicitly in 14, 28, or 128 speed-step format.

Important notes:

- address `0` is rejected,
- the direction bit is carried inside the `speed` byte,
- the library stores the locomotive state before queueing the packet,
- speed packets are repeated and also refreshed later.

When the rail power state is `ESTOP`, speed packets are stored for later refresh rather than being sent immediately as active drive commands.

### Reading back locomotive state

#### `getLocoData(uint16_t adr, uint8_t data[])`

Returns the currently stored locomotive data for the given address.

The returned array contains:

- `data[0]`: speed-step mode
- `data[1]`: speed byte including direction bit
- `data[2]`: function group including F0..F4 and stored F29..F31 bits
- `data[3]`: F5..F12
- `data[4]`: F13..F20
- `data[5]`: F21..F28

#### `getLocoDir(uint16_t adr)`

Returns the current stored direction bit.

#### `getLocoSpeed(uint16_t adr)`

Returns the current stored speed value without the direction bit.

### Function commands

The library supports several ways of controlling functions.

#### `setLocoFunc(address, type, fkt)`

Generic function control.

Parameters:

- `type`
  - `0`: off
  - `1`: on
  - `2`: toggle
  - `3`: invalid / error
- `fkt`: function number

Support for functions F0–F28 is always available.

Support for functions F29–F68 depends on a compile-time option.  
If the symbol `EXTENDFUNCTION` is defined in `DCCPacketScheduler.h`, functions up to F68 are supported.

#### `setFunctions0to4(address, functions)`
#### `setFunctions5to8(address, functions)`
#### `setFunctions9to12(address, functions)`
#### `setFunctions13to20(address, functions)`
#### `setFunctions21to28(address, functions)`
#### `setFunctions29to36(address, functions)`
#### `setFunctions37to44(address, functions)`
#### `setFunctions45to52(address, functions)`
#### `setFunctions53to60(address, functions)`
#### `setFunctions61to68(address, functions)`

These functions send complete DCC function groups. They are useful if the application already keeps the full bitfield for a function group.

Note that these methods are **not stateful**: you should pass the full desired state for the group each time you call them.

### Binary state functions

#### `setLocoFuncBinary(address, low, high)`

Sends a binary state control command. The code supports:

- short form for binary states below 128,
- long form up to 32767.

Note that binary state addresses below 29 are ignored.

### Reading back stored function groups

#### `getFunktion0to4(address)`
#### `getFunktion5to8(address)`
#### `getFunktion9to12(address)`
#### `getFunktion13to20(address)`
#### `getFunktion21to28(address)`
#### `getFunktion29to31(address)`

These return the stored function bits currently known to the scheduler.

---

## 4. Accessory commands

The library supports both **basic accessory packets** and **extended accessory packets**.

For basic accessories, the library also keeps track of the last known state internally.

### Basic accessories

#### `setBasicAccessoryPos(address, state)`

Sends a basic accessory command **with activation enabled**.

This is the standard and most commonly used function.  
It always sends a command with `activ = true`.


#### `setBasicAccessoryPos(address, state, activ)`

Sends a basic accessory command with explicit control over the activation bit.

Parameters:

- `address`: basic accessory address (0–2047)
- `state`: requested direction / position
- `activ`: controls the activation bit in the DCC packet  
  - `true`: activate the output (normal switching command)  
  - `false`: deactivate the output

### Notes:

- The first function is simply a shortcut for:  
  `setBasicAccessoryPos(address, state, true);`
- The library **does not automatically switch `activ` back to false**.
- Valid address range: **0 to 2047** (11-bit addressing)
- Addressing is automatically adapted to the selected turnout format (`ROCO` or `IB`)
- If implemented, the callback `notifyTrnt()` is called when a command is sent
- The last known accessory state is stored internally and can be retrieved


#### `getBasicAccessoryInfo(address)`

Returns the stored state of the given basic accessory.


### Extended accessories

#### `setExtAccessoryPos(address, state)`

Sends an extended accessory packet.

Additional details:

- Valid address range: **0 to 2047**
- If implemented, the callback `notifyExtTrnt()` is called when a command is sent

---

## 5. Service Mode commands

The library supports **Service Mode programming** on the programming track. Note that only **Direct Bit/Byte Mode** is handled.

When Service Mode is used, the scheduler switches the rail power state to `SERVICE`, sends reset packets, and then performs the requested programming sequence. During this process it can use a user-supplied current-sense callback to detect decoder acknowledge pulses.

### Direct Service Mode write

#### `opsProgDirectCV(uint16_t CV, uint8_t CV_data)`

Requests a Service Mode **write byte** operation.

### Direct Service Mode verify

#### `opsVerifyDirectCV(uint16_t CV, uint8_t CV_data)`

Requests a Service Mode **verify byte** operation.

### Direct Service Mode read

#### `opsReadDirectCV(uint16_t CV)`

Requests a Service Mode **read** operation.

The actual behavior depends on the configured read mode loaded from EEPROM:

- bit mode,
- byte mode,
- or both.

### Decoder reset packets

#### `opsDecoderReset(uint8_t repeat = RESET_CONT_REPEAT)`

Queues a broadcast decoder reset packet for Service Mode use.

### ACK handling and callbacks

During Service Mode reads and verifies, the scheduler can use the weak callback `notifyCurrentSense()` to measure current draw and detect acknowledge pulses from the decoder.

When a programming action completes:

- `notifyCVVerify(CV, value)` is called on success,
- `notifyCVNack(CV)` is called on failure.

These callbacks are weak symbols and can be implemented by the application.

---

## 6. Configuration (EEPROM) variables

The scheduler loads configuration values from EEPROM or equivalent persistent storage by calling:

### `loadEEPROMconfig()`

This happens automatically from `setup()`, but can also be called directly.

The following configuration locations are used by the code:

- `EEPROMRailCom` (`50`)
  Enables or disables RailCom cutout generation
  Valid values: `0` or `1`

- `EEPROMProgReadMode` (`53`)
  Service Mode read strategy
  Values used by the code:
  - `0`: none
  - `1`: bit
  - `2`: byte
  - `3`: both

- `EEPROMRSTsRepeat` (`60`)
  Repeat count for the initial reset packet sequence

- `EEPROMRSTcRepeat` (`61`)
  Repeat count for continuing reset packets

- `EEPROMProgRepeat` (`62`)
  Repeat count for programming packets

If stored values are outside the accepted range, the library writes default values back to storage.

The loaded configuration is then applied to the hardware and scheduler state:

- RailCom enable is forwarded to the waveform generator,
- programming read mode is stored,
- programming repeat counts are stored,
- Service Mode repeat count is forwarded to the hardware packet engine.

---

## 7. Programming on the Main (PoM)

The library supports the following PoM commands:

#### `opsProgramCV(address, CV, CV_data)`

PoM write byte.

#### `opsPOMwriteBit(address, CV, Bit_data)`

PoM bit write.

#### `opsPOMreadCV(address, CV)`

PoM read request.

These functions generate the corresponding PoM packets and place them in the programming queue. Whether and how the decoder responds depends on the decoder and the surrounding RailCom setup.

---

## 8. Direct calls using `dccPacketEngine`

Most users will only need the scheduler interface. However, the lower-level hardware object `dccPacketEngine` is also available and can be accessed directly for hardware-specific settings that are not exposed through the scheduler.

These calls are intended for advanced use.

### Signal and timing settings

#### `dccPacketEngine.setDccSignalInverted(bool inverted)`

Changes the polarity of the DCC output signal.

#### `dccPacketEngine.setPreambleLength(uint8_t value)`

Changes the normal DCC preamble length. The minimum accepted value is 17.

#### `dccPacketEngine.setPreambleLengthSM(uint8_t value)`

Changes the Service Mode preamble length. The minimum accepted value is 20.

### AUX output behavior

#### `dccPacketEngine.setAuxActiveLevel(bool activeHigh)`

Changes the active level of the AUX output.

#### `dccPacketEngine.setRailComGapInAux(bool useForRcCutout)`

Controls whether the AUX pin is used for the RailCom cutout signal.

### Hardware mode note

Note that the exact electrical behavior of `dccRailPin`, `dccRailAuxPin`, `dccMonitorPin`, `RunOutputSignal()`, and `StopOutputSignal()` depends on the selected hardware mode in the underlying driver layer.

---

## 9. Miscellaneous

### Weak callback hooks

The library offers several weak callback functions that an application may implement:

- `notifyLokAll(...)`
- `notifyTrnt(...)`
- `notifyExtTrnt(...)`
- `notifyCVVerify(...)`
- `notifyCVPOMRead(...)`
- `notifyRailpower(...)`
- `notifyCurrentSense()`
- `notifyCVNack(...)`

These hooks are intended for integration with user interfaces, current-sense hardware, feedback buses, or higher-level control software.

### Slot and accessory memory

The library stores:

- locomotive state in a slot array,
- basic accessory state in a bitmap.

The available capacity depends on platform RAM and compile-time limits, but those hardware-specific details are intentionally documented elsewhere.

### What the library sends to the rails

In practical use, the rail output behaves as follows:

- after startup, reset packets are sent first, followed by idle packets,
- under normal operation, locomotive and accessory packets are scheduled and refreshed,
- speed and function packets are repeated and then kept alive by periodic refresh,
- emergency stop can be broadcast to all decoders,
- Service Mode temporarily replaces normal rail traffic with programming/reset traffic,
- RailCom cutout can be enabled if supported by the hardware layer.

### Main application pattern

A typical application pattern is:

1. create a `DCCPacketScheduler` object,
2. call `setup(...)`,
3. send commands such as `setSpeed()`, `setLocoFunc()`, or `setBasicAccessoryPos()`,
4. call `update()` continuously from `loop()`.

Without step 4, packets will remain queued and will not continue flowing to the rails.
