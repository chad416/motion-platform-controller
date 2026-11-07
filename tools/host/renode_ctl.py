"""Minimal client for the Renode monitor socket (renode -P <port>)."""
from __future__ import annotations

import socket
import time


class RenodeMonitor:
    def __init__(self, host: str = "127.0.0.1", port: int = 4444, connect_timeout: float = 30.0):
        deadline = time.monotonic() + connect_timeout
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=5)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.5)
        self.sock.settimeout(0.2)

    def cmd(self, line: str, settle: float = 0.05) -> str:
        self.sock.sendall((line + "\n").encode())
        time.sleep(settle)
        out = b""
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                out += chunk
        except socket.timeout:
            pass
        return out.decode(errors="replace")

    def send(self, line: str) -> None:
        """Queue a command without waiting for its output (Renode runs them in order)."""
        self.sock.sendall((line + "\n").encode())

    def drain(self) -> None:
        try:
            while self.sock.recv(65536):
                pass
        except socket.timeout:
            pass

    def on(self, machine: str, line: str) -> None:
        """Run a command on one machine. Fire-and-forget: a few ms, not a socket timeout."""
        self.send(f'mach set "{machine}"')
        self.send(line)

    def close(self) -> None:
        self.sock.close()
