#!/bin/sh
# Usage: test-u.sh vtvorbis.orig|vtvorbis.fixed down|drop
# Uplink with -k -u to a local Icecast server. Uses -p (plain Ogg Vorbis),
# because stock Icecast rejects the vtvorbis timestamp stream.
# Scenario "down": server down at start, started at 12 s.
# Scenario "drop": server up at start, killed at 5 s, restarted at 12 s.
V=$1; S=$2
sed -e 's#<logdir>.*</logdir>#<logdir>/tmp</logdir>#' -e 's#<pidfile>.*</pidfile>##' /etc/icecast2/icecast.xml > /tmp/ice.xml
[ $S = drop ] && { icecast2 -b -c /tmp/ice.xml > /dev/null 2>&1; sleep 2; }
( vtgen -t -r 48000 -na=0.1 -T now,+120 | $V -e -p -k -u shout,127.0.0.1,8000,/test.ogg,hackme 2>/tmp/err; echo "exit status $?" > /tmp/status ) &
sleep 5
[ $S = drop ] && pkill icecast2
sleep 7
icecast2 -b -c /tmp/ice.xml > /dev/null 2>&1
sleep 45
printf 'GET /test.ogg HTTP/1.0\r\n\r\n' | timeout 8 nc 127.0.0.1 8000 | sed '1,/^\r$/d' > /tmp/live.ogg
echo "== $V, server $S"; cat /tmp/err; cat /tmp/status 2>/dev/null || echo "still running"
echo "bytes fetched from icecast at the end: $(wc -c < /tmp/live.ogg)"
vtvorbis.orig -d -p < /tmp/live.ogg 2>/dev/null | vtstat -i 2>&1 | grep -E 'samples|breaks'
