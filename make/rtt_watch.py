#!/usr/bin/env python3
"""
tools/rtt_watch.py — RTT heap/stack critical monitor for GOcontroll S1

Connects to SEGGER RTT over telnet (port 19021, exposed by JLinkGDBServer)
and shows ONLY heap/stack events. Every other line is silently dropped.

Usage (from the application/ directory):

  make gdbserver              # terminal 1 — keep running
  make rtt_watch              # terminal 2

  make rtt_watch AUTO=1       # single terminal — starts gdbserver automatically
  make rtt_watch AUTO=1 VERBOSE=1  # also show OK heap/stack values (green)

Direct invocation:
  python tools/rtt_watch.py [--auto] [--verbose]
  python tools/rtt_watch.py --heap-warn 8192 --stack-crit 8

Default thresholds:
  heap  < 4096 B  → yellow   heap  < 1024 B  → red + bell
  stack < 64 words → yellow  stack < 16 words → red + bell  (1 word = 4 bytes on CM33)
"""

import argparse
import re
import socket
import subprocess
import sys
import time
from datetime import datetime

# ── ANSI (disabled when stdout is not a TTY, e.g. piped to a file) ───────────
_TTY = sys.stdout.isatty()
def _c(s): return s if _TTY else ''

RED  = _c('\033[91m'); YEL = _c('\033[93m'); GRN = _c('\033[92m')
CYN  = _c('\033[96m'); DIM = _c('\033[2m');  RST = _c('\033[0m')
BOLD = _c('\033[1m')

# ── Patterns ──────────────────────────────────────────────────────────────────

# Heap free: "free heap: 12345", "heap free 1234", "xPortGetFreeHeapSize: 1234"
_HEAP_RE = re.compile(
    r'(?:free[\s_]?heap|heap[\s_]?free|xPortGetFreeHeapSize)[^0-9]*(\d+)',
    re.I,
)

# Stack high-water mark: "HWM: 64", "high water: 32", "uxTaskGetStackHighWaterMark: 128"
_HWM_RE = re.compile(
    r'(?:hwm|high[\s_]?water(?:[\s_]?mark)?|uxTaskGetStackHighWaterMark)[^0-9]*(\d+)',
    re.I,
)

# Hard failures — always show, highest priority
_FATAL_RE = re.compile(
    r'\b(?:'
    r'overflow|'
    r'pvPortMalloc[\s_]?fail(?:ed)?|malloc[\s_]?fail(?:ed)?|'
    r'assert(?:ion)?[\s_]?fail(?:ed)?|'
    r'hardfault|hard[\s_]fault|usage[\s_]fault|bus[\s_]fault|mem(?:ory)?[\s_]?fault|'
    r'watchdog|wdt[\s_]?reset|wdt[\s_]?expire'
    r')\b',
    re.I,
)

# Soft keywords — show dimmed so the user can spot patterns that need a regex
_SOFT_RE = re.compile(
    r'\b(?:warn(?:ing)?|error|heap|stack|hwm|high[\s_]water|low|critical|memory|fault)\b',
    re.I,
)


def _ts() -> str:
    return datetime.now().strftime('%H:%M:%S')


def _classify(line: str, args):
    """Return (level, styled_line) or (None, None) to suppress the line."""

    # Fatal — always show, red + bold + bell
    if _FATAL_RE.search(line):
        return 'fatal', f'{BOLD}{RED}[{_ts()}] FATAL  {line.strip()}{RST}'

    # Heap free value
    m = _HEAP_RE.search(line)
    if m:
        n = int(m.group(1))
        if n <= args.heap_crit:
            return 'crit', (
                f'[{_ts()}] {RED}HEAP  {n:,} B free  [CRITICAL]{RST}'
                f'  {DIM}{line.strip()}{RST}'
            )
        if n <= args.heap_warn:
            return 'warn', (
                f'[{_ts()}] {YEL}HEAP  {n:,} B free  [LOW]{RST}'
                f'  {DIM}{line.strip()}{RST}'
            )
        if args.verbose:
            return 'info', f'[{_ts()}] {GRN}HEAP  {n:,} B free{RST}'
        return None, None

    # Stack high-water mark
    m = _HWM_RE.search(line)
    if m:
        n = int(m.group(1))
        if n <= args.stack_crit:
            return 'crit', (
                f'[{_ts()}] {RED}STACK HWM {n}w ({n * 4}B)  [CRITICAL]{RST}'
                f'  {DIM}{line.strip()}{RST}'
            )
        if n <= args.stack_warn:
            return 'warn', (
                f'[{_ts()}] {YEL}STACK HWM {n}w ({n * 4}B)  [LOW]{RST}'
                f'  {DIM}{line.strip()}{RST}'
            )
        if args.verbose:
            return 'info', f'[{_ts()}] {GRN}STACK HWM {n}w ({n * 4}B){RST}'
        return None, None

    # Generic keyword match — show dimmed, useful while calibrating patterns
    if _SOFT_RE.search(line):
        return 'soft', f'[{_ts()}] {DIM}{line.strip()}{RST}'

    return None, None  # suppress — not interesting


def _recv_lines(sock: socket.socket):
    """Yield text lines from a raw RTT telnet socket, stripping TELNET IAC bytes."""
    buf = b''
    while True:
        data = sock.recv(4096)
        if not data:
            return
        # Strip TELNET IAC sequences (0xFF cmd opt) that JLink sometimes sends
        out = bytearray()
        i = 0
        while i < len(data):
            if data[i] == 0xFF and i + 2 < len(data):
                i += 3
            else:
                out.append(data[i])
                i += 1
        buf += bytes(out)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode('utf-8', errors='replace').rstrip('\r')
            if text:
                yield text


def _try_connect(host: str, port: int) -> 'socket.socket | None':
    try:
        return socket.create_connection((host, port), timeout=3)
    except OSError:
        return None


def _start_server(args) -> subprocess.Popen:
    cmd = [
        f'{args.jlink_dir}/JLinkGDBServerCL.exe',
        '-select', 'USB',
        '-device', args.device,
        '-if', args.jlink_if,
        '-speed', str(args.speed),
        '-port', '2331',
        '-RTTTelnetPort', str(args.port),
        '-rtos', 'GDBServer/RTOSPlugin_FreeRTOS',
    ]
    flags = subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == 'win32' else 0
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            creationflags=flags)


def main() -> None:
    ap = argparse.ArgumentParser(
        description='RTT heap/stack monitor for GOcontroll S1 (STM32H573RI + FreeRTOS)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Run `make gdbserver` in another terminal first, then `make rtt_watch`.',
    )
    ap.add_argument('--host',       default='localhost',              help='RTT telnet host')
    ap.add_argument('--port',       type=int, default=19021,          help='RTT telnet port')
    ap.add_argument('--heap-warn',  type=int, default=4096,  dest='heap_warn',  metavar='B')
    ap.add_argument('--heap-crit',  type=int, default=1024,  dest='heap_crit',  metavar='B')
    ap.add_argument('--stack-warn', type=int, default=64,    dest='stack_warn', metavar='WORDS')
    ap.add_argument('--stack-crit', type=int, default=16,    dest='stack_crit', metavar='WORDS')
    ap.add_argument('--verbose',    action='store_true', help='also show OK heap/stack values (green)')
    ap.add_argument('--auto',       action='store_true', help='start JLinkGDBServer automatically')
    ap.add_argument('--jlink-dir',  default='C:/Program Files/SEGGER/JLink_V876', dest='jlink_dir')
    ap.add_argument('--device',     default='STM32H573RI')
    ap.add_argument('--jlink-if',   default='SWD', dest='jlink_if')
    ap.add_argument('--speed',      type=int, default=4000)
    args = ap.parse_args()

    server_proc = None

    print(
        f'\n{BOLD}RTT Watch{RST}  {CYN}{args.device}{RST}\n'
        f'  heap  warn < {args.heap_warn:,}B   crit < {args.heap_crit:,}B\n'
        f'  stack warn < {args.stack_warn}w     crit < {args.stack_crit}w  (1w = 4B on CM33)\n'
        f'{DIM}  All other lines are hidden. Ctrl-C to stop.{RST}\n'
    )

    try:
        while True:
            sock = _try_connect(args.host, args.port)

            if sock is None:
                if args.auto and server_proc is None:
                    print(f'{DIM}[{_ts()}] Starting JLinkGDBServer on port {args.port}...{RST}')
                    server_proc = _start_server(args)
                    time.sleep(3)
                    sock = _try_connect(args.host, args.port)

                if sock is None:
                    if not args.auto:
                        print(
                            f'{RED}Cannot connect to {args.host}:{args.port}{RST}\n'
                            f'  Run {BOLD}make gdbserver{RST} in another terminal, then retry.\n'
                            f'  Or use {BOLD}make rtt_watch AUTO=1{RST} to start it automatically.'
                        )
                        sys.exit(1)
                    print(f'{YEL}[{_ts()}] Waiting for RTT server on :{args.port}...{RST}', end='\r')
                    time.sleep(2)
                    continue

            total = shown = 0
            print(f'{GRN}[{_ts()}] Connected — monitoring for heap/stack events{RST}')
            try:
                for line in _recv_lines(sock):
                    total += 1
                    level, styled = _classify(line, args)
                    if level is None:
                        continue
                    shown += 1
                    print(styled)
                    if level in ('fatal', 'crit') and _TTY:
                        sys.stdout.write('\a')
                        sys.stdout.flush()
            except OSError:
                pass
            finally:
                sock.close()

            print(f'{YEL}[{_ts()}] Disconnected  (lines seen: {total}, shown: {shown})  — reconnecting...{RST}')
            time.sleep(2)

    except KeyboardInterrupt:
        print(f'\n{DIM}Stopped.{RST}')
    finally:
        if server_proc:
            server_proc.terminate()


if __name__ == '__main__':
    main()
