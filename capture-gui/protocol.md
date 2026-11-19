# Capture GUI protocol and raw sample layout

The page talks to the USB CDC serial byte stream with binary frames. All multibyte integers are little endian. A request is:

```
C7 51 | command:u8 | sequence:u8 | payload_length:u16 | payload
```

The response repeats the two magic bytes, uses `command | 0x80`, echoes the sequence and has the same length field. The first response payload byte is a result code (`0` success; nonzero error); remaining bytes are operation data.

| Command | Payload | Successful response data |
|---|---|---|
| `01` Arm/resume | empty; sampling starts at the next non-SOF host transaction and uses its bus speed | empty |
| `02` Freeze | empty | empty |
| `03` Status | empty | state:u8, speed:u8, tail_incomplete:u8, sample_count:u32, sample_rate_hz:u32, data_size:u32, capacity:u32, freeze_on_ack:u8 |
| `04` Read | byte_offset:u32, max_bytes:u16 | raw capture bytes |
| `05` Clear | empty | empty |
| `06` ACK trigger | `enable:u8` | empty |

Status state is `0` idle, `1` armed, `2` frozen. `tail_incomplete` is set when the DMA snapshot stops without confirming USB EOP; the final packet may be truncated. `sample_count` is the count of two-bit samples; `data_size` and `capacity` are byte counts. The GUI polls status once per second while connected when no other command is in flight, so it notices an asynchronous error-triggered freeze and enables capture reading.

When ACK trigger is enabled, capture freezes at a DMA boundary after the host
accepts a CRC-valid IN DATA packet and sends ACK. The retained raw buffer still
includes the preceding bus history. The GUI filters the packet list to
ACK-completed IN transactions and packet details report the interval from DATA's
observed EOP SE0→J transition to ACK's starting J→K transition.

Raw capture bytes pack four chronological samples per byte, least-significant two-bit pair first. Each pair is `D+` in bit 0 and `D−` in bit 1: `00` SE0, `01` D+ high, `10` D− high, `11` SE1. Thus FS J=`01`, K=`10`; LS J=`10`, K=`01`. The sampler runs at 48 MHz for a full-speed capture or 6 MHz for a low-speed capture. The decoder uses the sample rate and the decoded packet speed to convert sample offsets to bit times.

A packet at the capture’s nominal bus speed has four samples per bit. An LS packet in a 48 MHz capture has 32 samples per bit. SOF samples remain in the continuous raw stream for timing, but SOF packets are omitted from the packet list. The capture engine's byte ordering and GPIO bit mapping must match this format.

Firmware integrations enable the host hooks with `USB_PACKET_CAPTURE_DEBUG=1`
and link `src/usb_capture.c`. Initialize the sampler after the host and call
`usb_capture_task()` regularly from the host core. Serve requests from the CDC
core using the capture control and snapshot functions in `src/usb_capture.h`.
