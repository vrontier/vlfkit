# Test image for the vtvorbis reconnect fix: the original vtvorbis (0.9p) as
# vtvorbis.orig, the one from c/ as vtvorbis.fixed, and a local Icecast server.
# Build from the repository root, after building the vlfkit-original image:
#   docker build -f c/tests/vtvorbis-reconnect/Dockerfile -t vlfkit-reconnect-test .
FROM vlfkit-original
USER root
RUN apt-get update && apt-get install -y --no-install-recommends icecast2 procps \
    && rm -rf /var/lib/apt/lists/* \
    && chmod a+r /etc/icecast2/icecast.xml
COPY c/vtvorbis.c /usr/src/vlfrx-tools-0.9p/vtvorbis.fixed.c
RUN cd /usr/src/vlfrx-tools-0.9p \
    && cp vtvorbis /usr/local/bin/vtvorbis.orig \
    && cp vtvorbis.fixed.c vtvorbis.c && make vtvorbis \
    && cp vtvorbis /usr/local/bin/vtvorbis.fixed
COPY c/tests/vtvorbis-reconnect/test-n.sh c/tests/vtvorbis-reconnect/test-u.sh /usr/local/share/vlfkit-tests/
USER vlf
