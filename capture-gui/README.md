# USB capture GUI

Use firmware that implements the [capture protocol](protocol.md). Serve this
directory from a secure origin. For local development, run:

```sh
python3 -m http.server 8000
```

Open `http://localhost:8000/` in a Web Serial capable browser, connect to the
CDC port, and use the capture controls. The matching firmware frame and sample
formats are described in [protocol.md](protocol.md).

Diagnostic console logging is off by default. Add `?debug=1` to the page URL to
enable USB Serial and command trace logs; `?debug=0` or omitting the parameter
keeps them disabled.

After fetching a capture, select a decoded packet to inspect its transaction in
the waveform panel. D+ and D− are drawn from the raw four-times samples; decoded
packet spans and token fields are overlaid. Use the zoom controls to inspect
individual bit transitions. The horizontal axis is in USB bit times.

After Arm / Resume, the GUI automatically fetches the capture when its state
changes from Armed to Frozen.

Use **Copy current capture** to copy any loaded snapshot, whether or not it
decoded successfully. The JSON includes the packed raw samples, sample-rate
metadata, and decoder results. If clipboard access is unavailable, it is
downloaded as a JSON file.

Select **Start continuous capture** to enable the IN DATA+ACK trigger and repeat
Arm → wait for freeze → Fetch. Every complete, error-free IN → DATA0/DATA1 → ACK
in a fetched snapshot contributes one latency measurement. The table separates
Full Speed and Low Speed. Captures without a qualifying transaction are counted
separately. Both the table and packet details measure from the observed SE0→J
transition of DATA's EOP to the J→K transition at the start of ACK. The decoder uses the
ACK packet speed and sampler rate to convert that interval to bit times.

The status text remains on one line in the controls bar; hover it to read the
full message when it is truncated.

**Stop continuous capture** keeps the collected statistics on screen and restores
the previous ACK trigger setting. Starting again clears the statistics. The
device is re-armed after each fetch, so only traffic in the captured snapshots
contributes to the results.
