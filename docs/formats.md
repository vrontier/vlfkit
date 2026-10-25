# Formats and protocols

This document specifies the data formats and protocols of vlfrx-tools 0.9p as the source code implements them. It is written from the code, not from memory or from the manual, and every statement points to the place in `original/vlfrx-tools-0.9p/` where it can be checked. It is the specification that the maintained build must keep and the rebuild must meet.

Status: first draft, 2026-10-25. Sections 1 to 5 are complete for what they cover; open questions are collected in section 7. Where the code does something that looks unintended, this document describes what the code does and says so; it does not correct it.

Terms used throughout: a *sample* is one value of one channel; a *frame* is one sample of every channel at the same instant; a *block* is a header followed by a number of frames; a *stream* is a sequence of blocks. *srcal* is the sample rate calibration factor: the actual sample rate is the nominal rate times srcal. A *PPS* is the one-pulse-per-second output of a GPS receiver, which `vtcard` uses to timestamp samples precisely.

## 1. Conventions

All binary structures are written as they lie in memory on the producing machine: packed (`__attribute__((packed))`), with no padding, and in the byte order of that machine. There is no byte swapping anywhere in the code. Every platform the toolkit runs on today (x86, x86-64, ARM in little-endian mode as on the Raspberry Pi) is little-endian, so in practice **all formats are little-endian**, and this specification defines them so. A big-endian machine running 0.9p would produce streams that no little-endian machine can read.

`int` in the structures below is 32 bits on every supported platform; `double` is IEEE 754 binary64, `float` binary32. `VT_init()` refuses to run if `float` is not 4 bytes, `double` not 8, `struct VT_BUFFER` not 32 or `struct VT_BLOCK` not 48 bytes (`vtlib.c:1368`).

## 2. The VT stream

A VT stream is an unbounded sequence of blocks. Files, pipes, stdin/stdout and TCP connections all carry exactly the same byte sequence; lock-free buffers hold the same blocks in shared memory (section 3). A reader can start anywhere: every block carries the full stream parameters and its own timestamp, so no stream header exists.

### 2.1 Block header (`struct VT_BLOCK`, 48 bytes, `vtlib.h:325`)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | int32 | `magic` | 27859 (`0x00006CD3`, bytes `D3 6C 00 00`), `MAGIC_BLK` in `vtlib.c:55` |
| 4 | uint32 | `flags` | Sample format and timestamp kind, section 2.2 |
| 8 | uint32 | `bsize` | Frames per block (capacity), section 2.4 |
| 12 | uint32 | `chans` | Channels per frame |
| 16 | uint32 | `sample_rate` | Nominal sample rate, Hz, integer |
| 20 | uint32 | `secs` | Timestamp of the first frame: seconds since 1970-01-01 00:00:00 UTC |
| 24 | uint32 | `nsec` | Timestamp of the first frame: nanoseconds, 0 to 999,999,999 |
| 28 | int32 | `valid` | 1 when the block is complete; used by lock-free buffers, always 1 in files |
| 32 | int32 | `frames` | Number of frames actually present, 1 to `bsize` |
| 36 | int32 | `spare` | Always written as 0 |
| 40 | double | `srcal` | Sample rate calibration factor; actual rate = `sample_rate * srcal` |

The header is followed by **`bsize` frames of data, always**, whatever `frames` says: the block length on the wire is fixed at `48 + bsize * chans * sample_size` bytes (`block_size()`, `vtlib.c:675`; `VT_release()` writes `vtfile->bs` bytes, `vtlib.c:313`). Only the first `frames` frames are meaningful. The rest is whatever was left in the writer's block buffer, usually data of the previous block; readers must ignore it, and reference comparisons must not include it. Short blocks occur at the end of a stream and before a timing break (`VT_set_timebase()`, `vtlib.c:357`).

Frames are interleaved: frame *i* of a block with *C* channels starts at sample index *i·C*, channel *c* (counting from 0) is at *i·C + c*.

### 2.2 Flags

| Bits | Value | Name | Meaning |
|---|---|---|---|
| 0 | 1 | `VTFLAG_RELT` | Timestamps are relative, not absolute UTC. No program in 0.9p sets it; only `vtstat` reads it (`vtstat.c:493`). `vtgen` lists an option `-z` for it in its usage text but does not implement it |
| 1 to 3 | 0 | `VTFLAG_FLOAT8` | Samples are 8-byte floats (the default) |
| | 2 | `VTFLAG_FLOAT4` | 4-byte floats |
| | 4 | `VTFLAG_INT1` | 1-byte signed integers |
| | 6 | `VTFLAG_INT2` | 2-byte signed integers |
| | 8 | `VTFLAG_INT4` | 4-byte signed integers |

The format is the field `flags & 0x0E` (`VTFLAG_FMTMASK`, `vtlib.h:346`) and is an enumeration, not a set of bits. Other bits are not defined; any other value of the format field makes `frame_size()` bail out (`vtlib.c:667`). All channels of a stream share one format.

### 2.3 Sample values

Inside the programs every sample is a `double`, nominally in the range -1.0 to +1.0. Float formats store the value directly. Integer formats scale by the largest positive value of the type (`vtlib.c:203` to `261` for writing, `774` to `815` for reading):

| Format | Write (double v → integer) | Read (integer n → double) |
|---|---|---|
| i1 | `v * 127`, clipped to -128..127, then truncated toward zero | `n / 127.0` |
| i2 | `lround(v * 32767)`, clipped to -32768..32767 (rounded to nearest) | `n / 32767.0` |
| i4 | `v * 2147483647`, clipped, then truncated toward zero | `n / 2147483647.0` |

The difference is in the code as it stands: i2 rounds, i1 and i4 truncate. The most negative integer reads back slightly below -1.0. A rebuilt tool must reproduce both behaviours exactly, since the reference outputs depend on them.

### 2.4 Block size

The writer chooses `bsize` from the sample rate when it opens an output (`choose_bsize()`, `vtlib.c:114`): 8192 frames for rates from 16,000 to 192,000 Hz; below 16,000 Hz, 8192 halved until there are at least 3 blocks per second; above 192,000 Hz, 8192 doubled (at most 262,144) until there are at most 25 blocks per second. Readers take `bsize` from the stream and must not assume this rule.

### 2.5 Timestamps

The timestamp of a block is the time of its first frame. The time of frame *i* within the block is

    T(i) = secs + nsec·10⁻⁹ + i / (sample_rate · srcal)

(`VT_get_timestamp()`, `vtlib.h:496`). The writer computes the header of each new block from the time base it was last given plus the number of frames written since then, divided by `srcal · sample_rate`, and rounds the fraction to the nearest nanosecond (`VT_next_write()`, `vtlib.c:288` to `292`).

A *timing break* is a discontinuity in the stream. A reader declares one when a block's timestamp differs by more than one sample period from where the previous block's timestamp, frame count and srcal say it should be (`VT_read_next()`, `vtlib.c:1253` to `1264`). Lock-free buffer readers also report a break (value 2) when they have been overrun (section 3). Programs query this through `VT_rbreak()`.

How the timestamp is held in memory depends on the platform, and this has visible consequences. Where `long double` has at least 12 bytes (x86 and x86-64: 80-bit extended precision; 64-bit ARM Linux: 128-bit) a timestamp is one `long double` of seconds. Elsewhere (macOS on Apple silicon, 32-bit Raspberry Pi OS) it is a pair of an `int32` of seconds and a `double` fraction (`USE_COMPOUND_TIMESTAMP`, `vtlib.h:30`). The two can round differently. Measured on 2026-10-25 with `vtgen -a1 -r 44100 -na=0.1 -T 2020-01-01_00:00:00.123456789,+10`, 4 of 54 block headers differ by 1 ns between Linux x86-64 and macOS arm64 (for example `nsec` 609533886 against 609533887 in the block starting at 1577836801). Signals whose phase is computed from the timestamp (`VT_phase()`, `vtlib.h:551`) differ accordingly. The rebuild should treat ±1 ns as equal when comparing against the original.

The in-memory seconds are signed 32-bit in several places (`timestamp_compose(int secs, …)`, `int secs` in `VT_parse_timestamp()`, `vtwrite.c` and others), although the header field is unsigned. In 0.9p, timestamps from 2038-01-19 03:14:08 UTC onward wrap around: on Linux x86-64, `vtgen -T 2038-01-19_03:14:10,+2 | vtstat -i` reports a start of 1970-01-01. The format itself is good until 2106-02-07 06:28:15 UTC, when the unsigned field overflows.

### 2.6 Reading and resynchronising

A reader learns all parameters from the first block header: it reads 48 bytes, takes `chans`, `flags`, `bsize` and `sample_rate`, and from then on reads whole blocks of fixed length (`VT_read_next()`, `vtlib.c:1114`). If a block from a file or pipe does not start with the magic number, the reader scans byte by byte for the four magic bytes, reads the rest of that header and skips its data (`resync()`, `vtlib.c:1070`). For a network input, a bad magic number ends the stream (or, with `++`, waits for a new connection). If `sample_rate` or `flags` change within a stream the reader bails out with "parameters changed".

### 2.7 Stream names

How a program interprets the name of an input or output (`VT_open_input()`, `vtlib.c:999`; `VT_open_output()`, `vtlib.c:684`; `VT_parse_chanspec()`, `vtlib.c:1572`):

| Name | Input | Output |
|---|---|---|
| `-` | stdin | stdout |
| `path` | file or named pipe | file (created, truncated) or named pipe |
| `@name` | lock-free buffer `name` | lock-free buffer `name`, created if necessary |
| `+port` | listen on TCP `port`, accept one connection | — |
| `++port` | as `+port`, listen again when the connection drops | — |
| `+host,port` | — | connect to TCP `host:port` |
| `++host,port` | — | as `+host,port`, reconnect every 5 s when the connection fails |

Input names may end in `:c1,c2,…` to select channels (counted from 1). Output names may carry comma-separated options: a sample format (`f8`, `f4`, `i4`, `i2`, `i1`) and a number, which is the buffer length in seconds for `@name` (default 10, allowed 2 to 120) and the port for `+host`. Examples from the manual: `@bar,20,i4`, `+bar,51300,i2`, `/tmp/foo:1,3`.

## 3. Lock-free buffers

A lock-free buffer is a ring of blocks in shared memory with one writer and any number of readers. `@name` is the POSIX shared memory object `/name` (`shm_open()`, which appears as `/dev/shm/name` on Linux); where POSIX shared memory is not available, a plain file `BUFDIR/name` is mapped instead, `BUFDIR` being `/tmp` (`vtlib.c:489` to `595`; `configure`). Buffers are local to one machine; they are not a network format, but tools of the rebuild that share a buffer with tools of the original on the same machine must use this layout.

The memory holds a 32-byte header followed by `nblocks` blocks, each laid out as in section 2.1 (header plus `bsize` frames).

### 3.1 Buffer header (`struct VT_BUFFER`, 32 bytes, `vtlib.h:310`)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | int32 | `magic` | 26374 (`MAGIC_BUF`), written last when the buffer is set up |
| 4 | uint32 | `flags` | As in section 2.2 |
| 8 | uint32 | `nblocks` | Number of blocks in the ring: `sample_rate · seconds / bsize`, integer division |
| 12 | uint32 | `bsize` | Frames per block |
| 16 | uint32 | `chans` | Channels per frame |
| 20 | int32 | `semkey` | Hash of the name; a leftover of a semaphore scheme that is compiled out (`USE_SEM 0`) |
| 24 | uint32 | `load` | Index of the block the writer is filling now |
| 28 | uint32 | `sample_rate` | Nominal sample rate |

### 3.2 Protocol

Writer (`create_buffer()`, `vtlib.c:597`; `VT_release()`, `vtlib.c:318`): on creation it sets every block's `valid` to 0, `srcal` to 1 and `magic` to `MAGIC_BLK`, keeps `load` modulo the new `nblocks` (a restarted writer continues where the previous one stopped), and writes `magic` last. It fills the block at index `load` in place, sets its `frames` and `valid = 1`, advances `load` by one modulo `nblocks`, and sets `valid = 0` on the block now at `load`. The writer never waits for readers.

Reader (`open_buffer()`, `vtlib.c:954`; `VT_read_next()`, `vtlib.c:1156`): it starts at the block the writer is filling (`readp = load`). It waits while the block at `readp` is not valid or its `secs` is older than the last block it read, sleeping one block duration (`bsize / sample_rate` seconds) between checks, then takes the block and advances `readp`. If the new block's `secs` is more than 2 s after the previous one, the reader has been overrun and reports a timing break with value 2.

The code relies on `volatile` alone for ordering; there are no memory barriers. On x86 this is sufficient. Whether a reader on a multi-core ARM machine (Raspberry Pi) can see `valid = 1` before the sample data is an open question for the maintained build (section 7).

## 4. Network streams

A `+`/`++` network stream is a plain TCP connection over IPv4 carrying exactly the bytes of section 2, with no handshake and no framing of its own (`net_connect()`, `net_listen_accept()`, `vtlib.c:132` to `197`). The manual states that it is compatible with netcat: `vtcat @source +bar,41720` is the same as `vtcat @source | nc bar 41720`. The sending side always connects and the receiving side always listens. With `++`, the sender retries every 5 s and the receiver listens again 1 s after a connection drops; parameters must be the same after reconnecting. Host names are resolved with `gethostbyname()`, so IPv6 is not supported.

## 5. vtvorbis: compressed streams with timestamps

`vtvorbis -e` compresses a VT stream with Ogg Vorbis; `vtvorbis -d` turns it back into a VT stream. Vorbis is lossy, so the samples that come out are not those that went in, but the timestamps and srcal survive, because vtvorbis multiplexes a second logical stream of timing packets into the Ogg container (`vtvorbis.c`). This is the format in which stations share live data, and which the Listening Observatory reads.

### 5.1 Container

The output is an Ogg physical bitstream (RFC 3533) with two logical streams (`run_encode()`, `vtvorbis.c:298`):

- serial number 1: the timestamp stream, packets described below;
- serial number 0: a standard Vorbis I audio stream, `chans` channels at the nominal `sample_rate`, with the comment `encoded by vtvorbis`.

The decoder does not rely on the serial numbers: it classifies each new logical stream by trying its first packet as a Vorbis header and then as a `VT_OGG_HDR`, and ignores any other stream (`run_decode()`, `vtvorbis.c:873`). With `-p` the encoder writes plain Ogg Vorbis without the timestamp stream, and the decoder makes up timestamps from the local clock at the time of connection.

The encoder writes, in this order: the `VT_OGG_HDR` packet flushed on its own page (beginning of stream, granule position 0); the three Vorbis header packets; then audio, with a `VT_OGG_BLK` packet at the start of every VT input block, its pages flushed before the audio of that block is submitted (`vtvorbis.c:419` to `516`). The default encoding is CBR at 64 kbit/s per channel; `-b` sets another CBR rate, `-q` selects VBR with a quality of -0.1 to 1.0, `-i` disables channel coupling.

### 5.2 Timestamp stream packets

Both packets are packed little-endian structures, like the VT block header.

`VT_OGG_HDR`, 16 bytes, first packet of the timestamp stream (`vtvorbis.c:70`):

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | int32 | `magic` | `0x1996C3A3` |
| 4 | int32 | `chans` | Channels |
| 8 | int32 | `rate` | Nominal sample rate |
| 12 | int32 | `spare` | 0 |

`VT_OGG_BLK`, 64 bytes, one per VT input block (`vtvorbis.c:84`):

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | int32 | `magic` | `0x4F60F817` |
| 4 | int32 | `tbreak` | Timing break flag of the input at this block (0, 1 or 2, see section 2.5) |
| 8 | uint64 | `posn` | Index of the audio frame this timestamp applies to, counted from 0 at the start of the encoded stream; also the packet's granule position |
| 16 | int32 | `spare1` | 0 |
| 20 | int32 | `spare2` | 0 |
| 24 | int32 | `secs` | Timestamp of frame `posn`, seconds since 1970 UTC (signed here) |
| 28 | int32 | `nsec` | Nanoseconds; the source comment says microseconds but the value is nanoseconds |
| 32 | double | `srcal` | Sample rate calibration factor |
| 40 | double | `glat` | Always 0 in 0.9p |
| 48 | double | `glong` | Always 0 in 0.9p |
| 56 | double | `gasl` | Always 300 in 0.9p |

Note that the encoder truncates the fraction to whole nanoseconds (`vtvorbis.c:493`), whereas the VT block writer rounds (section 2.5); a timestamp can therefore lose up to 1 ns on its way through vtvorbis.

### 5.3 Decoding timestamps

The decoder keeps the received `VT_OGG_BLK` records in a queue and counts decoded audio frames from 0. At the start of each VT output block it takes the most recent record whose `posn` is not beyond the current frame count *n* and computes the block's timestamp as `secs + nsec·10⁻⁹ + (n − posn) / (srcal · rate)` (`tq_get()`, `vtvorbis.c:583`). It reports a break when timestamps jump by more than one sample, and discards audio whose timestamp would go backwards.

### 5.4 Transport

The encoder writes the Ogg stream to stdout, to a plain TCP connection with `-n`, or to an Icecast server through libshout with `-u shout,server,port,mount,password` (HTTP, Ogg format, user `source`); the decoder reads stdin or a TCP connection given with `-n`. Unlike the `+` stream names, `-n` works in both directions on both sides: `-n port` listens, `-n host,port` connects (`parse_net()`, `net_setup()`, `vtvorbis.c:1031`, `258`). TCP keepalive is set to 5 s idle, 5 s interval, 3 probes where the platform defines `SOL_TCP` (`set_keepalive()`, `vtvorbis.c:169`); that is Linux, not macOS. With `-k`: a connecting side retries every 5 s; an encoder whose connection fails discards input for 20 s before it starts again, at a VT block boundary; a decoder treats 30 s without data as a dropped connection and starts again after 5 s (`vtvorbis.c:322` to `347`, `620` to `647`, `887` to `904`).

## 6. Storage: vtwrite and vtread

`vtwrite` stores a stream as a directory of files, `vtread` reads it back (`vtwrite.c`, `vtread.c`). Each file is a plain sequence of VT blocks as in section 2, written exactly as received. A file's name is the UTC time of its first block in the form `YYMMDD-HHMMSS` (two-digit year, read back as 20YY by `VT_timestamp_from_filename()`, `vtlib.c:1822`). A new file is started when the block's time enters a new period of `-G` seconds (default 86400, periods counted from 1970-01-01) and after every timing break.

## 7. Open questions

- Memory ordering in lock-free buffers on ARM (section 3.2): to be tested on a Raspberry Pi with a fast writer and several readers, and settled in the maintained build.
- Text output formats (`vtraw -oa`, `vtstat -i`, the result files of `vtsid`, `vttoga`, `vtspot` and others) are also interfaces that station scripts depend on; they are to be specified tool by tool when each tool is rebuilt.
- The `vtsid` configuration file and data files, and the `vtevent` and `vtspot` output files, are not yet covered.
