# The maintained C build

This directory is the maintained build of vlfrx-tools, roadmap step 2 of vlfkit. It started as an exact copy of Paul Nicholson's release 0.9p (`original/vlfrx-tools-0.9p/`), and every change since then is a commit of its own, so `git log -- c/` and `git diff` against `original/` show exactly what was changed and why. The changes are kept small: only what is needed to build and run on current Linux and Raspberry Pi OS (and Windows through WSL2), and fixes for problems that stations run into.

Build and install it as described for the original in `docs/building-the-original.md`, from this directory instead of the tarball:

    cd c
    ./configure
    make
    sudo make install

## Changes against 0.9p

- **vtvorbis: reconnects after a dropped uplink instead of exiting.** With `-n` or `-u`, a dropped connection made vtvorbis exit silently with status 0 before `-k` could reconnect, because the library ends programs quietly on SIGPIPE. vtvorbis now ignores SIGPIPE while sending to a server, logs the failure and, with `-k`, reconnects. It also no longer crashes when the server cannot be reached at start-up with `-k`, retries a failed host name lookup with `-k`, and logs send errors without `-v`. For stations on 0.9p that don't want to switch builds, the change is also available as a patch: `c/patches/0001-vtvorbis-reconnect.patch` (see below). Tests: `c/tests/vtvorbis-reconnect/`.

## Patches for 0.9p

`c/patches/` holds changes that stations running 0.9p can apply to their own source tree:

    cd vlfrx-tools-0.9p
    patch -p1 < 0001-vtvorbis-reconnect.patch
    make vtvorbis
    sudo install -p vtvorbis /usr/local/bin/

The patches are written against 0.9p; on older releases they may not apply unchanged.
