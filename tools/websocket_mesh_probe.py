#!/usr/bin/env python3
"""Probe the Web bridge using only Python's standard library.

Mesh mode validates the 64-byte MMSH packet header used on port 9003.  Status
mode reads and decodes one or more JSON text frames from port 9002.  The probe
performs a real RFC 6455 handshake and therefore checks more than a TCP listen.
"""

import argparse
import base64
import hashlib
import json
import os
import socket
import struct
from pathlib import Path
from typing import Dict, Tuple
from urllib.parse import urlparse


MESH_MAGIC = 0x4D4D5348
MESH_VERSION = 1
MESH_HEADER = struct.Struct("<IHBBQQ10I")


def receive_exact(connection: socket.socket, size: int) -> bytes:
    chunks = []
    received = 0
    while received < size:
        chunk = connection.recv(size - received)
        if not chunk:
            raise RuntimeError("WebSocket connection closed before the frame completed")
        chunks.append(chunk)
        received += len(chunk)
    return b"".join(chunks)


def connect(url: str, timeout_s: float) -> Tuple[socket.socket, bytes]:
    parsed = urlparse(url)
    if parsed.scheme != "ws" or not parsed.hostname:
        raise ValueError("only ws:// URLs are supported")
    port = parsed.port or 80
    path = parsed.path or "/"
    if parsed.query:
        path += "?" + parsed.query
    connection = socket.create_connection((parsed.hostname, port), timeout=timeout_s)
    connection.settimeout(timeout_s)
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {parsed.hostname}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    connection.sendall(request.encode("ascii"))
    response = b""
    while b"\r\n\r\n" not in response:
        response += connection.recv(4096)
        if len(response) > 65536:
            raise RuntimeError("oversized HTTP upgrade response")
    headers, remainder = response.split(b"\r\n\r\n", 1)
    lines = headers.decode("iso-8859-1").split("\r\n")
    if " 101 " not in lines[0]:
        raise RuntimeError(f"WebSocket upgrade failed: {lines[0]}")
    header_map = {}
    for line in lines[1:]:
        if ":" in line:
            name, value = line.split(":", 1)
            header_map[name.strip().lower()] = value.strip()
    expected_accept = base64.b64encode(
        hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
    ).decode("ascii")
    if header_map.get("sec-websocket-accept") != expected_accept:
        raise RuntimeError("server returned an invalid Sec-WebSocket-Accept")
    # A WebSocket server normally starts frames after the upgrade response.  In
    # practice this bridge does not coalesce them, but retain any bytes if it does.
    return connection, remainder


class FrameReader:
    def __init__(self, connection: socket.socket, remainder: bytes = b""):
        self.connection = connection
        self.buffer = remainder

    def exact(self, size: int) -> bytes:
        if len(self.buffer) >= size:
            result, self.buffer = self.buffer[:size], self.buffer[size:]
            return result
        prefix, self.buffer = self.buffer, b""
        return prefix + receive_exact(self.connection, size - len(prefix))

    def frame(self) -> Tuple[int, bytes]:
        first, second = self.exact(2)
        final = bool(first & 0x80)
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", self.exact(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self.exact(8))[0]
        mask = self.exact(4) if masked else b""
        payload = self.exact(length)
        if masked:
            payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        if not final:
            raise RuntimeError("fragmented WebSocket messages are not supported by this probe")
        return opcode, payload


def parse_mesh(payload: bytes) -> Dict[str, object]:
    if len(payload) < MESH_HEADER.size:
        raise RuntimeError("mesh packet is shorter than its 64-byte header")
    unpacked = MESH_HEADER.unpack_from(payload)
    (
        magic,
        version,
        operation,
        flags,
        revision,
        stamp_ns,
        vertex_count,
        index_count,
        frame_id_offset,
        frame_id_bytes,
        positions_offset,
        colors_offset,
        indices_offset,
        packet_bytes,
        reserved0,
        reserved1,
    ) = unpacked
    if magic != MESH_MAGIC or version != MESH_VERSION:
        raise RuntimeError("mesh magic/version mismatch")
    if packet_bytes != len(payload):
        raise RuntimeError("mesh packet_bytes does not match the WebSocket payload")
    if frame_id_offset + frame_id_bytes > len(payload):
        raise RuntimeError("mesh frame_id range exceeds the packet")
    frame_id = payload[frame_id_offset:frame_id_offset + frame_id_bytes].decode("utf-8")
    if operation == 1:
        if vertex_count == 0 or vertex_count % 3 != 0:
            raise RuntimeError("replace packet has an invalid triangle-list vertex count")
        if positions_offset + vertex_count * 12 > len(payload):
            raise RuntimeError("mesh position range exceeds the packet")
        if colors_offset + vertex_count * 4 > len(payload):
            raise RuntimeError("mesh color range exceeds the packet")
    elif operation == 2:
        if vertex_count or index_count:
            raise RuntimeError("clear packet contains geometry")
    else:
        raise RuntimeError(f"unknown mesh operation: {operation}")
    return {
        "operation": "replace" if operation == 1 else "clear",
        "flags": flags,
        "revision": revision,
        "stamp_ns": stamp_ns,
        "frame_id": frame_id,
        "vertex_count": vertex_count,
        "triangle_count": index_count // 3 if index_count else vertex_count // 3,
        "index_count": index_count,
        "packet_bytes": packet_bytes,
        "positions_offset": positions_offset,
        "colors_offset": colors_offset,
        "indices_offset": indices_offset,
        "reserved_zero": reserved0 == 0 and reserved1 == 0,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url")
    parser.add_argument("--kind", choices=("mesh", "status"), default="mesh")
    parser.add_argument("--messages", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.messages <= 0 or args.timeout <= 0.0:
        raise SystemExit("--messages and --timeout must be positive")
    connection, remainder = connect(args.url, args.timeout)
    reader = FrameReader(connection, remainder)
    results = []
    try:
        while len(results) < args.messages:
            opcode, payload = reader.frame()
            if opcode in (8,):
                raise RuntimeError("server closed the WebSocket")
            if opcode in (9, 10):
                continue
            if args.kind == "mesh":
                if opcode != 2:
                    raise RuntimeError(f"expected binary mesh frame, got opcode {opcode}")
                results.append(parse_mesh(payload))
            else:
                if opcode != 1:
                    raise RuntimeError(f"expected text status frame, got opcode {opcode}")
                results.append(json.loads(payload.decode("utf-8")))
    finally:
        connection.close()
    rendered = json.dumps({"url": args.url, "kind": args.kind, "messages": results}, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
