# Test: vtvorbis reconnects after a dropped uplink

Compares the vtvorbis of 0.9p (`vtvorbis.orig`) with the one in `c/` (`vtvorbis.fixed`) when the connection to the receiving server drops. Build the images from the repository root:

    docker build -f docker/original/Dockerfile -t vlfkit-original .
    docker build -f c/tests/vtvorbis-reconnect/Dockerfile -t vlfkit-reconnect-test .

Then run each test with both programs:

    T=/usr/local/share/vlfkit-tests
    docker run --rm vlfkit-reconnect-test sh $T/test-n.sh vtvorbis.orig
    docker run --rm vlfkit-reconnect-test sh $T/test-n.sh vtvorbis.fixed
    docker run --rm vlfkit-reconnect-test sh $T/test-u.sh vtvorbis.orig drop
    docker run --rm vlfkit-reconnect-test sh $T/test-u.sh vtvorbis.fixed drop

`test-n.sh` uplinks with `-k -n` to a listener that drops after 4 s and comes back at 8 s. `test-u.sh` uplinks with `-k -u` to a local Icecast server that is either down at the start (`down`) or killed after 5 s (`drop`), and is (re)started at 12 s; it then fetches the stream from Icecast and decodes it. `test-u.sh` uses `-p` (plain Ogg Vorbis), because stock Icecast refuses the vtvorbis timestamp stream ("Seen BOS page with unknown type"); the reconnect code is the same.

Results on 2026-10-25 (Debian 13, arm64):

| Test | vtvorbis.orig | vtvorbis.fixed |
|---|---|---|
| `test-n.sh` | exits silently, status 0; nothing sent after the drop | logs "network send failed: Broken pipe", reconnects, 214 KB received |
| `test-u.sh … drop` | exits silently, status 0 | reconnects; the stream from Icecast decodes, 0 timing breaks |
| `test-u.sh … down` | retries, with one needless failure cycle | connects when the server comes up; stream decodes |
| `-u` with server down at start, timestamp stream (no `-p`) | segmentation fault | keeps retrying, connects when the server comes up |
| encode and decode through stdout, no network | | output byte-identical to the original |
