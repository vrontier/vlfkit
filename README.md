# vlfrx

vlfrx continues Paul Nicholson's vlfrx-tools, the VLF Receiver Software Toolkit: the original kept intact, kept building, and rebuilt for the next twenty years.

## Mission

vlfrx-tools is the software behind a large part of amateur VLF radio. For more than a decade, receiving stations around the world have used it to capture signals with precise timestamps, stream them across the network, and study them: whistlers and sferics, SID monitoring, TOGA measurements, the amateur bands around 8 kHz, and much more. Paul Nicholson wrote and maintained it alone, together with EbNaut and many other tools, and shared all of it under an open license.

Paul has passed away. His website may not stay online, and software that nobody maintains stops building on new systems after a few years. This project exists so that his work stays available and usable.

We commit to three things:

1. **Preserve.** Paul's last release, 0.9p, is kept here exactly as he published it, together with its manual and web pages. It is never edited; it is the reference for everything else.
2. **Maintain.** A maintained build of the C toolkit, with only the changes needed to build and run on current Linux, macOS and the Raspberry Pi. Stations that depend on vlfrx-tools today should have a version that keeps working.
3. **Rebuild.** A new implementation, tool by tool, that is compatible with the original: the same stream formats, the same network protocol, the same timestamps, so that old and new tools can be mixed in one pipeline. The programs keep their names (`vtcard`, `vtfilter`, `vtspec` and so on), so that existing station scripts keep working. Every rebuilt tool is checked against the output of the original before it replaces it.

What we will not do: change the meaning of a format or a timestamp, remove a capability the original had without saying so, or present Paul's work as our own. His name and copyright stay on everything derived from it.

## What vlfrx-tools is

In Paul's words: a modular software toolkit for timestamped signal processing and storage. It is command line driven and built for scripted processing, with precision timestamped capture and processing, streamable packeted signal streams, distributed pipelines with built-in networking, whistler detection, a signal database, TOGA measurement, and SID monitoring with phase and bearing. It runs in real time, for post-processing or for simulations, on PCs, the Raspberry Pi and the BeagleBone, and reads sound cards, SDR-IQ and RTL2832U receivers. Besides VLF, it serves meteor forward scatter, seismographic and natural radioactivity recording, ELF and magnetometers, radio astronomy, bat detection and amateur radio.

Release 0.9p consists of about 50 small programs (`vtcard`, `vtfilter`, `vtspec`, `vtsid`, `vtvorbis`, `vtspot`, `vttoga` and others) that pass a timestamped sample stream to one another through pipes and share one library, `vtlib`. It is about 38,000 lines of C and depends on FFTW, libvorbis, libpng, ALSA or OSS, librtlsdr, SoapySDR and XForms.

## Repository layout

| Path | Content |
|---|---|
| `original/vlfrx-tools-0.9p/` | The source tree of release 0.9p, unchanged |
| `original/vlfrx-tools-0.9p.tgz` | The release tarball as published |
| `original/vlfrx-tools-0.9p.pdf` | The manual for 0.9p |
| `original/website/` | The project's web pages: overview, full documentation (`notes.html`), `vtsid.conf` reference |
| `docs/provenance.md` | Where every file came from, with checksums |

Coming: `c/` for the maintained C build, a directory for the new implementation, and `tests/` with reference outputs recorded from the original.

## Roadmap

1. **Reference.** Build 0.9p as it is on Linux. Record reference outputs: known inputs (synthetic signals from `vtgen`, recordings, live network streams) through the main tools, with their outputs stored as test data.
2. **Maintained C build.** Make the C toolkit build cleanly on current Linux, macOS and Raspberry Pi OS. Every change is small, documented and tested against the reference outputs.
3. **Document the formats.** Write down the stream format, the buffer format and the vtvorbis network protocol from the source, as the specification the new implementation must meet.
4. **Rebuild.** Port `vtlib` first, then the most-used tools, one at a time, each checked against the reference outputs. Rust is proposed as the language (real-time signal processing, predictable timing, memory safety, easy cross-compilation for the Raspberry Pi); the decision is still open. The XForms interface of `vtscope` will not be ported but replaced.
5. **Hardware.** Sound card input, SDR input and GPS timing (`vtcard`, `vtsoapy`, `vtrtlsdr`, `vtubx`) come last, tested on a Raspberry Pi.

Related work by Paul, such as EbNaut and LCnetgen, is not part of this repository for now.

## Status

Step 0: the original is imported. Nothing has been built or changed yet.

## Name

vlfrx stands for VLF receiver, after the title of Paul's project, the VLF Receiver Software Toolkit. vlfrx-tools refers to his original; vlfrx is this continuation of it.

## License

vlfrx-tools is released under the Simplified (2-clause) BSD license, copyright Paul Nicholson; see `LICENSE`. The maintained build and the new implementation are released under the same license, and keep his copyright notice next to that of their contributors.

## Credits

vlfrx-tools was written by Paul Nicholson. The original project page was http://abelian.org/vlfrx-tools/.
