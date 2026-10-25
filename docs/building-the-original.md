# Building and running the original 0.9p

This page belongs to roadmap step 1, the reference. It explains how to build Paul Nicholson's release 0.9p exactly as he published it, run its test suite and use its programs. The easiest way is a Docker container: it needs no libraries on your machine, gives everybody the same build, and works on Linux, macOS and Windows. A build without Docker, on Debian or Ubuntu, is described at the end.

The image is built from `original/vlfrx-tools-0.9p.tgz`, the tarball as Paul released it; its SHA-256 checksum is checked against `docs/provenance.md` before it is unpacked, and nothing in it is changed. What we found when building 0.9p on different systems is recorded in `docs/build-notes.md`.

## What you need

- Docker: Docker Engine on Linux, Docker Desktop on macOS and Windows.
- A copy of this repository.
- About 1 GB of disk space for the image.

The image builds natively on x86-64 and on 64-bit ARM (Apple silicon, a Raspberry Pi with 64-bit Raspberry Pi OS).

## Build the image

From the root of the repository:

    docker build -f docker/original/Dockerfile -t vlfkit-original .

This takes a few minutes the first time; the image is about 800 MB. It installs the libraries, checks and unpacks the tarball, runs `./configure --with-rtlsdr --with-sdriq`, `make` and `make install`. The programs end up in `/usr/local/bin` inside the image; the build logs stay in `/usr/src/vlfrx-tools-0.9p/` (`configure.log`, `make.log`).

The base system is Debian 13 (trixie), the base of current Raspberry Pi OS. Another can be chosen with `--build-arg`, for example:

    docker build -f docker/original/Dockerfile --build-arg BASE=ubuntu:24.04 -t vlfkit-original:ubuntu .

To build for 32-bit Raspberry Pi OS on another machine, add `--platform linux/arm/v7`. Docker then emulates an ARM processor, which works but is many times slower.

## Check the build

Paul's test suite:

    docker run --rm vlfkit-original sh -c 'cd test && ./runall'

It ends with `All tests passed`. The suite doesn't cover lock-free buffers, network streams or `vtvorbis`, so the image has a short extra check for these:

    docker run --rm --shm-size=64m vlfkit-original vlfkit-smoke-test

Each of its three checks prints the number of samples and timing breaks, followed by the expected values.

`vtstat -V` shows the version: `vtlib version 0.9p`.

## Use the programs

An interactive shell with all programs on the `PATH`:

    docker run --rm -it vlfkit-original

Every program prints its options with `-?`, for example `vtgen -?`. The full documentation is `original/website/notes.html` and the manual `original/vlfrx-tools-0.9p.pdf`.

### Files

Mount a directory of your machine into the container, here the current directory as `/data`:

    docker run --rm -v "$PWD":/data vlfkit-original vtgen -r 48000 -s a=0.3,f=1000 -T 2020-01-01,+10 /data/sine.vt
    docker run --rm -v "$PWD":/data vlfkit-original vtstat -i /data/sine.vt

On Linux, add `--user "$(id -u):$(id -g)"` so that the files you write belong to you and the container may write into the directory. Docker Desktop on macOS and Windows takes care of this itself.

### Pipes

With `-i`, a container reads its standard input from your shell, so pipelines can mix the container with programs on your machine, or chain several containers:

    docker run --rm -i vlfkit-original vtstat -i < sine.vt
    docker run --rm vlfkit-original vtgen -r 8000 -na=0.1 -T 2020-01-01,+1 | docker run --rm -i vlfkit-original vtraw -oa | head

For longer pipelines it is simpler to run them inside one container: `docker run --rm vlfkit-original sh -c 'vtgen … | vtfilter … | vtstat -i'`.

### Lock-free buffers

Buffers (`@name`) live in shared memory, which Docker limits to 64 MB per container by default. A 10-second buffer of two channels at 48 kHz in the default 8-byte format needs about 8 MB; give larger buffers more room with `--shm-size`, for example `--shm-size=512m`.

A buffer is visible only inside its own container unless containers share their shared memory. Start the writer with `--ipc=shareable` and a name, and the readers with `--ipc=container:<name>`:

    docker run -d --rm --name vlfsrc --ipc=shareable --shm-size=256m vlfkit-original \
        vtgen -t -r 48000 -s a=0.3,f=1000 -T now,+60 @raw
    docker run --rm --ipc=container:vlfsrc vlfkit-original sh -c 'vtcat -E 5 @raw - | vtstat -i'

### Network streams

To receive a stream with a listening program (`+port`, or `vtvorbis -d -n port`), publish the port with `-p`:

    docker run --rm -p 41720:41720 vlfkit-original vtstat -i +41720

A sending program connects as usual (`+host,port`). From one container to a listening program on the same machine, use the host name `host.docker.internal` on Docker Desktop. On Linux, `--network host` lets the container use the machine's network directly, which is also the simplest way to receive a live vtvorbis stream:

    docker run --rm --network host -v "$PWD":/data vlfkit-original \
        sh -c 'vtvorbis -d -n server,port | vtcat -E 60 - /data/live.vt'

Replace `server,port` with the address of the stream.

### What the image doesn't do

- `vtsoapy` isn't built: it doesn't compile against current SoapySDR (`docs/build-notes.md`, problem 5). `vtrtlsdr` and `vtsdriq` are built.
- Receiver hardware hasn't been tried in the container. On Linux, `--device /dev/snd` (sound card for `vtcard`), `--device /dev/bus/usb` (RTL-SDR) and `--device /dev/ttyACM0` (u-blox GPS for `vtubx`) should pass devices through, but this is untested. For a real station, install the programs directly on the machine (see below) rather than in a container.
- `vtscope` and `vtspec` are built but need an X11 display, which the image doesn't set up.
- The plotting scripts (`vtplot`, `vtsgram`, `vtsidplot` and others) need gnuplot, sox and ImageMagick, which aren't installed in the image.

## Without Docker, on Debian, Ubuntu or Raspberry Pi OS

Install the build tools and libraries:

    sudo apt-get install build-essential autoconf libfftw3-dev libvorbis-dev libogg-dev \
        libpng-dev libasound2-dev librtlsdr-dev libsoapysdr-dev libforms-dev libxpm-dev \
        libx11-dev libsamplerate0-dev libflac-dev libshout-dev libncurses-dev

Then unpack, build and test (`--with-rtlsdr --with-sdriq` only if you need SDR input):

    tar xzf original/vlfrx-tools-0.9p.tgz -C /tmp
    cd /tmp/vlfrx-tools-0.9p
    ./configure --with-rtlsdr --with-sdriq
    make
    sudo make install
    cd test && ./runall

`make install` puts the programs into `/usr/local/bin`. This works unchanged on Ubuntu 24.04 and 26.04 and on Debian 13, on x86-64 and on 32-bit and 64-bit ARM, the bases of current Raspberry Pi OS (tested in containers, not yet on a real Raspberry Pi; see `docs/build-notes.md`).
