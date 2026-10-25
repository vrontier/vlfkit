# vlfkit

vlfkit continues Paul Nicholson's vlfrx-tools, the VLF Receiver Software Toolkit: the original kept intact, kept building, and rebuilt into the future.

## Mission

vlfrx-tools is the software behind a large part of amateur VLF radio. For more than a decade, receiving stations around the world have used it to capture signals with precise timestamps, stream them across the network, and study them: whistlers and sferics, SID monitoring, TOGA measurements, the amateur bands around 8 kHz, and much more. Paul Nicholson wrote and maintained it alone, together with EbNaut and many other tools, and shared all of it under an open license.

Paul has passed away. His website may not stay online, and software that nobody maintains stops building on new systems after a few years. This project exists so that his work stays available and usable.

We commit to three things:

1. **Preserve.** Paul's last release, 0.9p, is kept here exactly as he published it, together with its manual and web pages. It is never edited; it is the reference for everything else.
2. **Maintain.** A maintained build of the C toolkit, with only the changes needed to build and run on current Linux and the Raspberry Pi, and on Windows through WSL2. Stations that depend on vlfrx-tools today should have a version that keeps working.
3. **Rebuild.** A new implementation, tool by tool, that runs natively on Linux, macOS and Windows and is compatible with the original: the same stream formats, the same network protocol, the same timestamps, so that old and new tools can be mixed in one pipeline. The programs keep their names (`vtcard`, `vtfilter`, `vtspec` and so on), so that existing station scripts keep working. Every rebuilt tool is checked against the output of the original before it replaces it.

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
| `docs/building-the-original.md` | How to build, test and run 0.9p unchanged, with Docker or directly |
| `docs/build-notes.md` | What building 0.9p on current systems showed, and the problems found |
| `docs/formats.md` | The stream, buffer, network and vtvorbis formats, written from the source |
| `docker/original/` | Dockerfile and smoke test for the unchanged 0.9p build |

Coming: `c/` for the maintained C build, a directory for the new implementation, and `tests/` with reference outputs recorded from the original.

## Roadmap

1. **Reference.** Build 0.9p as it is on Linux; [docs/building-the-original.md](docs/building-the-original.md) explains how, with Docker or directly on Debian, Ubuntu and Raspberry Pi OS. Record reference outputs: known inputs (synthetic signals from `vtgen`, recordings, live network streams) through the main tools, with their outputs stored as test data.
2. **Maintained C build.** Make the C toolkit build cleanly on current Linux and Raspberry Pi OS, where the stations run. Every change is small, documented and tested against the reference outputs. macOS is not a target of the maintained build: 0.9p builds there unchanged with Homebrew GCC (not with Apple's compiler) and passes its tests, except for sound card input, but this is not supported; the rebuild runs natively on macOS. On Windows, the C build runs in WSL2, the Linux environment built into Windows; a native Windows port of the C code would mean rewriting the Unix-specific parts (pipes, shared memory, serial ports, ALSA/OSS) and is left to the rebuild.
3. **Document the formats.** Write down the stream format, the buffer format and the vtvorbis network protocol from the source, as the specification the new implementation must meet.
4. **Rebuild.** Port `vtlib` first, then the most-used tools, one at a time, each checked against the reference outputs. Rust is proposed as the language (real-time signal processing, predictable timing, memory safety, easy cross-compilation for the Raspberry Pi, native builds for Linux, macOS and Windows); the decision is still open. The new implementation runs natively on Linux, macOS and Windows: sound, network and file access go through cross-platform libraries, and the Unix-specific parts, which 0.9p concentrates in `vtlib`, are designed for all three systems from the start. The XForms interface of `vtscope` will not be ported but replaced.
5. **Hardware.** Sound card input, SDR input and GPS timing (`vtcard`, `vtsoapy`, `vtrtlsdr`, `vtubx`) come last, tested on a Raspberry Pi.

Related work by Paul, such as EbNaut and LCnetgen, is not part of this repository for now.

## Status

Step 1 under way. The original is imported. 0.9p builds unchanged and passes its tests on current Linux (x86-64, and 32-bit and 64-bit ARM in containers); the problems found are in `docs/build-notes.md`. The first draft of `docs/formats.md` is written. Reference outputs are not yet recorded, and nothing in the code has been changed.

## Name

vlfkit stands for VLF toolkit. It echoes the title of Paul's project, the VLF Receiver Software Toolkit, and says plainly what the project is, without taking his project's name. vlfrx-tools refers to his original; vlfkit is this continuation of it.

The name vlfrx itself was considered and set aside: it is already used by other projects, among them a Python library on PyPI and a VLF receiver for the Teensy microcontroller, and a third project of the same name would confuse people looking for any of them. vlfkit was free on GitHub, crates.io and PyPI when it was chosen.

## License

vlfrx-tools is released under the Simplified (2-clause) BSD license, copyright Paul Nicholson; see `LICENSE`. We redistribute his manual and web pages under the same license; `docs/provenance.md` explains why. The maintained build and the new implementation are released under the same license, and keep his copyright notice next to that of their contributors.

## Credits

vlfrx-tools was written by Paul Nicholson. The original project page was http://abelian.org/vlfrx-tools/.
