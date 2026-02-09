# DCC Encoding on an ESP32

Several approaches can be used to generate a DCC signal on the ESP32 using its RMT peripheral. In the following sections, we outline a number of these approaches and explain the design decisions we have made.

It is worth noting that in ESP-IDF v5 (the current major release), Espressif has completely redesigned the RMT driver. While DCC signal generation in ESP-IDF v4 resulted in a very clean and relatively straightforward implementation, achieving the same result in ESP-IDF v5 is considerably more challenging.

## ESP-IDF v4 (DCC-EX)

ESP-IDF v4 provides a continuous streaming, hardware ring-buffer model for the ESP32 RMT peripheral. At the time of writing (February 2026), the DCC-EX implementation builds on this model to generate the DCC waveform. It keeps the RMT in permanent loop mode while the CPU refills the hardware buffer as needed.

#### Precomputed waveforms

Before transmission starts, DCC-EX prepares three arrays of rmt_item32_t in normal RAM:

- a preamble buffer containing a full DCC preamble plus start bit and an RMT end-of-transfer marker;
- an idle buffer containing a valid DCC idle pattern (not silence), used whenever no user packet is ready;
- a single reusable data buffer into which each DCC packet is converted from bytes into RMT timing symbols.

All conversion from DCC bytes to RMT symbols happens once, in normal application context, well before the data is handed to the RMT hardware.

#### Streaming via a hardware ring buffer

At startup the channel is prefilled with the preamble followed by the idle pattern. From that moment on, the RMT operates autonomously in loop mode.

Whenever the hardware reaches the end of the currently loaded data, it generates a TX-end interrupt. Inside that ISR, DCC-EX directly appends new items to the same running RMT memory. If a new packet is available, the ISR writes the precomputed data waveform into the buffer; otherwise it writes idle symbols.

Crucially, the ISR never stops or restarts the RMT transmitter. It only adds new data to the existing stream. The RMT memory therefore behaves like a hardware ring buffer that software keeps continuously filled in real time. This yields a nearly perfect DCC signal.

-----

## ESP-IDF v5
In ESP-IDF v5, Espressif completely rewrote the RMT driver. The new API is job-based, centered around `rmt_transmit()` calls, software transmit queues, and encoder objects.

In ESP-IDF v5, there is no public equivalent of rmt_fill_tx_items(), and application code can no longer refill a running RMT FIFO from an ISR. Consequently, the ring-buffer-based streaming model used by DCC-EX in ESP-IDF v4 cannot be directly reproduced in ESP-IDF v5.
Although it may be technically possible to recreate such a mechanism by writing directly to the RMT registers and bypassing the ESP-IDF driver, doing so would tightly couple the code to the hardware and significantly reduce portability.

As a consequence, ESP-IDF v5 implementations generally work with queued transmissions, typically using ping-pong buffers, rather than incremental hardware refilling of the circular RMT buffer.

#### Encoding approach
In ESP-IDF v5 the RMT peripheral is driven through the `rmt_transmit()` driver call. Each call hands a complete DCC packet to the driver, either as raw bytes processed by a DCC encoder, or as precomputed RMT symbols.

When the TX channel is created, it is typically configured with a software transmit queue of depth two (`trans_queue_depth = 2`). This allows the driver to hold two pending transmissions at the same time: one actively being streamed to the RMT hardware, and one waiting in the driver’s internal queue. During initialization it is common to enqueue two packets (often idle packets) so that the channel starts immediately and remains continuously supplied with data.

When the active transmission finishes, the RMT hardware raises a `RMT_TX_END` event that is handled inside the RMT driver in ISR context. Within this ISR context the RMT driver automatically advances to the second queued transmission and begins streaming it to hardware without any application intervention. Only afterwards a new packet is submitted via an application `rmt_transmit()` call to refill the driver queue.

The result is a rolling two-slot pipeline: while one packet is on the wire, a second is already queued inside the driver, and a third is prepared in application code.

#### Gap between subsequent DCC packets

Measurements using a logic analyzer revealed that there is always a gap of tens of microseconds between the end of a DCC packet (in queue 1) and the start of the next DCC packet (in queue 2).

This is illustrated in the figure below. As can be seen, the last symbol of the DCC packet (*thus the second half of the DCC end bit*) lasts **73 µs**, instead of the expected **58 µs** (the lines **A**, **B** and **C** are explained below).

[![gap without compensation](RMT-Offset-No-Compensation.png)](RMT-Offset-No-Compensation.png)


Analysis of the Espressif RMT driver code revealed the following cause for this problem:
- Each call to `rmt_transmit()` queues a packet in the software transmit queue; the first begins immediately, while the second waits.
- When the first packet finishes (**A**), the RMT hardware generates a `RMT_TX_END` interrupt.
- Upon entering the RMT ISR (**B**), any registered callback (e.g., `rmtDoneIsr`) is executed first, in ISR context.
- After the callback, the ISR checks the software queue and begins the next packet if one is available, all still within the same ISR.
- The encoder, whose handle is passed as a parameter to rmt_transmit(), fills the RMT hardware buffer with symbols, after which transmission starts (**C**).

Taken together, these steps consume a significant amount of time, resulting in a gap of several tens of microseconds between consecutive DCC packets. This is too long to be acceptable for DCC signal generation.

Fortunately, this time is largely deterministic. The only non-deterministic element is the latency from the hardware raising the `RMT_TX_END` interrupt, to the CPU starting the ISR (*in the figure above: the time between **A** and* **B**). Such latency will increase if higher or equal priority ISRs are already running, resulting in jitter. The chances to experience jitter can fortunately be reduced by giving the RMT ISR a higher priority, for example:

```tx_chan_config.intr_priority = 1; // higher priority than default 0```

#### Compensating for the inter-packet gap

To compensate for the largely deterministic gap between consecutive DCC packets, the last symbol in an RMT stream can be shortened by an amount that corresponds to this gap. As shown in the figure below, in our code this value was 17 µs. This value will be different for other implementations, and depends to a large extent on the choice of *encoder* used. For example, OpenRemise uses a dedicated ZIMO DCC encoder and applies a compensation of 31–34 µs.

[![gap with compensation](RMT-Offset-With-Compensation.png)](RMT-Offset-With-Compensation.png)

#### Encoder

An important choice is which encoder (handle) is passed as a parameter to the `rmt_transmit()` call. The main tasks of an encoder are:
- translating input data into RMT symbols, and
- refilling the 64-item RMT hardware buffer when the input data exceeds this length.

OpenRemise uses the (open source) ZIMO RMT DCC encoder. In that approach, the input to `rmt_transmit()` is a sequence of bits representing the DCC signal (including preamble, start, and end bits). Architecturally, using a dedicated DCC encoder is sound and clean, but it introduces an extra delay when calling `rmt_transmit()`.

Our code, by contrast, uses the standard `copy_encoder`, which expects ready-made RMT symbols as input. This requires that DCC bits are encoded into RMT symbols **before** calling `rmt_transmit()`. This difference in encoder choice explains the observed gap times: 17 µs for our code versus 31–34 µs for OpenRemise/ZIMO.

## RailCom gap

At first glance, it may seem appealing to use a second, dedicated RMT channel to output a RailCom gap signal on a separate pin. However, in practice this is not feasible, as explained in detail in [RMT Two Channels](RMT-two-channels.md)
. Each RMT channel requires a noticeable internal setup time before its first symbol can be transmitted. In our measurements, we observed an initial offset of approximately 17 µs.

Compensating for this initial misalignment, similar to the approach used for the inter-packet gap, does not provide a reliable solution. Once both channels are running, every time their internal FIFO queues switch, the second channel consistently starts noticeably later than the first. Continuously aligning both channels would require compensating multiple symbols for each channel, which is complex and error-prone. Therefore, generating perfectly synchronized DCC signals on two independent RMT channels is impractical.

To generate a dedicated RailCom gap signal, our implementation, like OpenRemise, will use a standard ESP32 timer rather than a second RMT channel.
