# DCC Encoding on STM32 (STM32Duino)

This STM32 implementation targets the **STM32F4xx** and **STM32H7xx** families and is intended for use with the STM32Duino (Arduino) core.

## Supported processors and hardware requirements

This implementation supports:

- STM32F4xx series  
- STM32H7xx series  

The DCC output is generated using a hardware timer channel. As a consequence, the pin assignment is fixed:

- **DCC output (rail signal): PA6** (TIM3 channel 1, alternate function)
- **AUX output (enable / RailCom cutout): configurable pin**

The AUX pin is used to:
- enable or disable the external H-bridge, and
- optionally generate the RailCom cutout signal.

### Important notes

- On **STM32H7**, the source code requires disabling the data cache at startup:

      SCB_DisableDCache();

  This must be placed as the first line in `setup()`.

- STM32H7 devices do not provide native EEPROM. The source code requires that EEPROM access is replaced by flash-based emulation.  
  For this purpose, the **FlashStorage_STM32** library must be installed:  
  https://github.com/khoih-prog/FlashStorage_STM32

- GPIO usage is partly constrained by the timer hardware. In particular, the DCC output pin (PA6) is fixed due to the use of TIM3 channel 1.

---

## Implementation details

### Timer-based DCC signal generation

The DCC waveform is generated using **TIM3**, configured as:

- 16-bit timer  
- running at **1 MHz** (1 µs resolution)  
- **output compare toggle mode** on channel 1  

Each time the timer reaches the value in `CCR1`, the output pin toggles. This produces the DCC waveform.

Instead of storing relative durations, the DMA buffers contain **absolute compare values**. Each new value is computed by adding the required half-bit time to the previous compare value. This allows continuous operation across timer overflows without special handling.

DCC timing used in this implementation:

- Logical "1": two half-bits of **58 µs**  
- Logical "0": two half-bits of **100 µs** (according to RCN-210)  

The timer runs continuously; it is never stopped between packets.

---

### DMA-driven waveform streaming

A **DMA channel** is used to feed new compare values into `TIM3->CCR1`.

- Peripheral: `TIM3->CCR1`  
- Transfer size: 16-bit  
- Direction: memory → peripheral  

Two DMA buffers are used:

- `DMAMesg_Buf1`  
- `DMAMesg_Buf2`  

While one buffer is being transmitted by DMA, the other buffer is prepared with the next packet.

At the end of a DMA transfer:

1. The DMA ISR switches to the other buffer  
2. The DMA stream is restarted immediately  
3. The freed buffer is filled with the next packet  

This ensures a continuous DCC stream without gaps.

---

### Packet encoding into DMA buffers

Packets are translated into sequences of compare values before transmission.

Each packet consists of:

- optional RailCom cutout  
- preamble bits  
- start bits  
- data bits  
- end bit  

The encoding process converts each DCC bit into two half-bit intervals and stores the corresponding absolute compare times in the DMA buffer.

---

### RailCom cutout (AUX pin)

The RailCom cutout on the AUX pin is generated using **TIM4**, also running at 1 MHz.

The timing sequence consists of:

1. A short pre-delay after the DCC packet boundary  
2. Activation of the cutout (AUX toggled)  
3. A second timer interval defining the cutout duration  
4. Restoration of the AUX signal  

The pre-delay compensates for the time between DMA completion and the actual signal transition.

---

### STM32F4-specific configuration

For STM32F4:

- Timer:
  - TIM3 → DCC signal  
  - TIM4 → RailCom cutout timing  

- DMA:
  - DMA1 Stream 4  
  - Channel select: **TIM3_CH1**  

- Timer clock:
  - Derived from APB1  
  - Prescaled to exactly 1 MHz  

- Interrupt priorities:
  - DMA interrupt: high priority  
  - TIM4 interrupt: medium priority  
  - SysTick: lowest priority  

---

### STM32H7-specific configuration

For STM32H7:

- Timer usage is identical:
  - TIM3 → DCC signal  
  - TIM4 → RailCom cutout  

- DMA:
  - DMA1 Stream 4  
  - Request routed via **DMAMUX**  
  - Request ID: TIM3_CH1  

- Additional requirements:
  - Cache coherency must be handled explicitly  
  - DMA buffers must be cleaned before enabling DMA  

---

## Signal quality

The **DCC signal** is generated entirely by hardware (timer + DMA). Once a buffer is active, no CPU interaction is required for individual bit timing. As a result, the DCC waveform has stable timing and does not exhibit software-induced jitter.

The **RailCom cutout on the AUX pin** is controlled by a timer that is started from an interrupt. Because of this, its timing may show a small amount of variation due to interrupt latency and compensation delays.

In practice, this means:

- DCC signal timing is deterministic  
- RailCom cutout timing is accurate, but may have minor jitter  
