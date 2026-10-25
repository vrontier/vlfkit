# The maintained C build

This directory is the maintained build of vlfrx-tools, roadmap step 2 of vlfkit. It started as an exact copy of Paul Nicholson's release 0.9p (`original/vlfrx-tools-0.9p/`), and every change since then is a commit of its own, so `git log -- c/` and `git diff` against `original/` show exactly what was changed and why. The changes are kept small: only what is needed to build and run on current Linux and Raspberry Pi OS (and Windows through WSL2), and fixes for problems that stations run into.

Build and install it as described for the original in `docs/building-the-original.md`, from this directory instead of the tarball:

    cd c
    ./configure
    make
    sudo make install

## Changes against 0.9p

None yet.
