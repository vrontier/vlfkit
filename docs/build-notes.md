# Building 0.9p unchanged

This is the record of building release 0.9p exactly as Paul published it, on the systems vlfkit wants to support, and of every problem found. Nothing under `original/` was changed; the problems listed here are what the maintained C build will have to address. Each build unpacked `original/vlfrx-tools-0.9p.tgz` into a fresh directory, ran `./configure` and `make -k`, then ran Paul's test suite (`test/runall`) with the freshly built programs first in `PATH`.

First round: 2026-10-25.

## Results

| System | Compiler | Built | Tests | Notes |
|---|---|---|---|---|
| Ubuntu 24.04 LTS, x86-64 (container on `max`) | gcc 13.3 | all 50 programs | all pass | 1 warning (libshout deprecation) |
| Debian 13 trixie, x86-64 (container on `max`) | gcc 14.2 | all 50 programs | all pass | 1 warning |
| Ubuntu 26.04 LTS, x86-64 (container on `max`) | gcc 15.2 | all 50 programs | all pass | 2 warnings |
| Debian 13 trixie, arm64 (container on the Mac; the base of 64-bit Raspberry Pi OS) | gcc 14.2 | all 50 programs | all pass | 1 warning |
| Debian 13 trixie, armhf (emulated container; the base of 32-bit Raspberry Pi OS) | gcc 14.2 | all 50 programs | all pass | 1 warning; emulated with QEMU, not yet confirmed on a real Pi |
| macOS 27.0.1, Apple silicon | Apple clang | 8 shell scripts only, no C program | — | see problems 1 to 3 |
| macOS 27.0.1, Apple silicon | Homebrew gcc 16.2, `--without-x11` | all except `vtcard`, `vtscope`, `vtspec` | all pass (t9 in test1 skipped) | see problems 3 and 4 |
| Windows 11, WSL2 | — | not tried | — | see below |

"All 50 programs" means the default set from `configure` plus `vtcard`, `vtscope`, `vtspec`, `vtstat`, `vtvorbis`, `vtflac` and `vtresample`, which `configure` enables when it finds their libraries. The SDR inputs are off by default and need `--with-rtlsdr`, `--with-soapy`, `--with-sdriq`; with those on Ubuntu 24.04, `vtrtlsdr` and `vtsdriq` build and `vtsoapy` does not (problem 5).

Paul's test suite covers `vtgen`, `vtcat`, `vtstat`, `vtraw`, `vtwrite`, `vtread`, `vtnspec`, `vtsid`, `vtsidex`, `vtcmp`, `vtfilter`, `vtresample`, `vtmix`, `vttoga`, `vtspot`, `vtdate`, `vtdata`, `vtmatch` and `vtsl`. It does not touch lock-free buffers, network streams or `vtvorbis`, so these three were checked by hand on every system where the build succeeded: a 4-second buffer written in real time by `vtgen -t` and read by `vtcat`, a TCP stream from `vtgen` to `vtstat -i +port`, and a round trip `vtgen | vtvorbis -e | vtvorbis -d | vtstat -i`. All three worked everywhere, with no timing breaks and the expected number of samples. The first decoded samples of the vtvorbis round trip were identical on Linux x86-64, Linux arm64 and macOS.

## Problems found

macOS is not a target of the maintained build (decided 2026-10-25): stations run Linux, mostly on the Raspberry Pi, and the rebuild will run natively on macOS. Problems 1 to 4 are therefore recorded for reference only and will not be fixed in the maintained build; 0.9p builds on macOS unchanged with Homebrew GCC, as described above.

1. **macOS: nested functions.** Seven programs (`vtcat`, `vtfilter`, `vtpolar`, `vttime`, `vttoga`, `vtspot`, `vtubx`) define functions inside functions, a GCC extension that clang does not support. Apple's compiler therefore cannot build them. Homebrew's GCC can, and does. The maintained build can either require GCC on macOS or move these functions out to file scope; the second is a mechanical change and also helps any future clang or BSD build.
2. **macOS: `-allow_stack_execute`.** When the compiler accepts `-fnested-functions` (old Apple GCC did), `configure` adds the linker option `-allow_stack_execute`, which today's Apple linker rejects, so even programs without nested functions fail to link with Apple clang.
3. **macOS: no sound input.** `vtcard` supports ALSA and OSS only, and macOS has neither; it fails to compile. `configure --without-soundcard` leaves it out. A Core Audio input belongs to the rebuild, not the maintained build. XForms is not available from Homebrew either, so `vtscope` and `vtspec` need `--without-x11`.
4. **macOS: include paths.** The Makefile passes `CFLAGS` but not `CPPFLAGS` to the compiler, so Homebrew's headers must be given as `CFLAGS="-O2 -I/opt/homebrew/include" LDFLAGS="-L/opt/homebrew/lib"`. Minor, but it should be documented or fixed.
5. **SoapySDR 0.8 and later: `vtsoapy` does not compile.** `vtsoapy.c:264` selects, with `#if 1`, the call form of `SoapySDRDevice_setupStream()` from SoapySDR 0.7; the comment above it says "for later version", but since 0.8 (2021) the function returns the stream instead. Paul's `#else` branch already has the 0.8 form. The fix is to choose by `SOAPY_SDR_API_VERSION`. It can be tested with Mike's HackRF One.
6. **Year 2038.** The stream format stores seconds as an unsigned 32-bit number (good until 2106), but the code handles them as signed `int` in several places (`timestamp_compose()`, `VT_parse_timestamp()`, `vtwrite.c` and others). On Linux x86-64, `vtgen -T 2038-01-19_03:14:10,+2 | vtstat -i` reports a start of 1970-01-01, and `vtraw` prints negative times after 2038-01-19 03:14:07 UTC. Twelve years away, but stations keep archives, and the fix belongs in the maintained build.
7. **libshout.** `shout_set_format()` is deprecated in favour of `shout_set_content_format()`; only a warning today.
8. **Results differ slightly between platforms.** The same command does not produce byte-identical output on Linux x86-64 and macOS arm64. Block timestamps can differ by 1 ns, because x86-64 keeps timestamps in 80-bit `long double` while macOS on Apple silicon and 32-bit ARM use Paul's fallback of integer seconds plus a `double` fraction (see `docs/formats.md`, section 2.5). Sample values differ by about 1e-16 after `vtfilter` (different floating-point operation order, FFTW code paths and fused multiply-add) and by up to 4.5e-7 for signals whose phase is computed from the timestamp. Reference comparisons must therefore use tolerances, not byte equality.
9. **Lock-free buffers on ARM, to be checked.** The buffer code orders its writes with `volatile` alone, without memory barriers (`docs/formats.md`, section 3.2). That is safe on x86 but not guaranteed on multi-core ARM. The smoke test passed in an arm64 container, which proves little; it needs a stress test on a real Raspberry Pi.

## Windows through WSL2

Not tried yet: no Windows machine was at hand. WSL2 runs a real Linux kernel with an Ubuntu userland, so the Ubuntu 24.04 result should carry over for everything that works with files, pipes, buffers (`/dev/shm` exists in WSL2) and the network. The open points are sound card input (WSL2 offers PulseAudio through WSLg, not direct ALSA hardware access), USB SDR receivers (they need `usbipd-win` to pass the device through) and serial GPS receivers for `vtubx`. Inside WSL2, TCP streams reach other machines through Windows networking, which by default uses NAT, so a listening `vtvorbis` or `+port` inside WSL2 needs a port forward or mirrored networking.

## How the builds were run

Linux x86-64 builds ran in Docker containers on `max`, so that nothing was installed on the host: images from `ubuntu:24.04`, `ubuntu:26.04` and `debian:trixie` with `build-essential autoconf libfftw3-dev libvorbis-dev libogg-dev libpng-dev libasound2-dev librtlsdr-dev libsoapysdr-dev libforms-dev libxpm-dev libx11-dev libsamplerate0-dev libflac-dev libshout-dev libncurses-dev`. ARM builds ran in the same containers on the Mac under Docker Desktop (`--platform linux/arm64` and `linux/arm/v7`). The macOS build used Homebrew `fftw libsamplerate librtlsdr soapysdr libshout libvorbis flac libpng` and, for the second attempt, `gcc`. The Dockerfile and scripts will move into the repository with the reference-output harness.
