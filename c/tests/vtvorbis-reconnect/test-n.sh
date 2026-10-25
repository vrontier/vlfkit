#!/bin/sh
# Usage: test-n.sh vtvorbis.orig|vtvorbis.fixed
# Uplink with -k -n to a listener that drops after 4 s and comes back at 8 s.
V=$1
( timeout 4 nc -l -p 41800 > /dev/null ) &
sleep 1
( vtgen -t -r 48000 -na=0.1 -T now,+60 | $V -e -k -n 127.0.0.1,41800 2>/tmp/err; echo "exit status $?" > /tmp/status ) &
sleep 7
timeout 40 nc -l -p 41800 > /tmp/second.ogg
sleep 1
echo "== $V"; cat /tmp/err; cat /tmp/status 2>/dev/null || echo "still running after 48 s"
echo "bytes received after reconnect: $(wc -c < /tmp/second.ogg)"
