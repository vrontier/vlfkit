# vlfrx-tools 0.9p, built unchanged from Paul Nicholson's release tarball.
#
# Build from the repository root:
#   docker build -f docker/original/Dockerfile -t vlfkit-original .
# See docs/building-the-original.md for how to use the image.

ARG BASE=debian:trixie
FROM ${BASE}

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential autoconf ca-certificates file bc gawk netcat-openbsd \
        libfftw3-dev libvorbis-dev libogg-dev libpng-dev libasound2-dev \
        librtlsdr-dev libsoapysdr-dev libforms-dev libxpm-dev libx11-dev \
        libsamplerate0-dev libflac-dev libshout-dev libncurses-dev \
    && rm -rf /var/lib/apt/lists/*

# The tarball is checked against the checksum recorded in docs/provenance.md
# before it is unpacked, so the image is built from exactly what Paul published.
COPY original/vlfrx-tools-0.9p.tgz /usr/src/
RUN cd /usr/src \
    && echo "b6af64d6d9cfe9b9a4a8e758d4e917ab3be8c11904bbbd4e0bd0ffbcb3f28ed7  vlfrx-tools-0.9p.tgz" | sha256sum -c - \
    && tar xzf vlfrx-tools-0.9p.tgz

# vtsoapy is left out: it does not compile against SoapySDR 0.8 and later
# (docs/build-notes.md, problem 5).
RUN cd /usr/src/vlfrx-tools-0.9p \
    && ./configure --with-rtlsdr --with-sdriq > configure.log 2>&1 \
    && make -j"$(nproc)" > make.log 2>&1 \
    && make install > install.log 2>&1

COPY docker/original/smoke-test /usr/local/bin/vlfkit-smoke-test

RUN useradd -m vlf \
    && cp -r /usr/src/vlfrx-tools-0.9p/test /home/vlf/test \
    && chown -R vlf:vlf /home/vlf/test \
    && chmod 755 /usr/local/bin/vlfkit-smoke-test
USER vlf
WORKDIR /home/vlf
CMD ["bash"]
