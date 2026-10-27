"""Tiny HTTP + WebSocket server on asyncio, standard library only (RFC 6455 subset)."""
import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
from urllib.parse import urlsplit, parse_qs

WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


class WSClient:
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer
        self.closed = False
        self._send_lock = asyncio.Lock()

    async def send_text(self, text: str):
        await self._send(0x1, text.encode('utf-8'))

    async def _send(self, opcode, payload: bytes):
        if self.closed:
            return
        n = len(payload)
        if n < 126:
            header = struct.pack('!BB', 0x80 | opcode, n)
        elif n < 65536:
            header = struct.pack('!BBH', 0x80 | opcode, 126, n)
        else:
            header = struct.pack('!BBQ', 0x80 | opcode, 127, n)
        async with self._send_lock:
            try:
                self.writer.write(header + payload)
                await self.writer.drain()
            except (ConnectionError, RuntimeError):
                self.closed = True

    async def recv(self):
        """Returns a text message, or None when the connection closes."""
        buf = b''
        while True:
            try:
                b1, b2 = await self.reader.readexactly(2)
                n = b2 & 0x7F
                if n == 126:
                    n = struct.unpack('!H', await self.reader.readexactly(2))[0]
                elif n == 127:
                    n = struct.unpack('!Q', await self.reader.readexactly(8))[0]
                mask = await self.reader.readexactly(4) if b2 & 0x80 else b'\0\0\0\0'
                data = bytearray(await self.reader.readexactly(n))
            except (asyncio.IncompleteReadError, ConnectionError):
                self.closed = True
                return None
            for i in range(n):
                data[i] ^= mask[i % 4]
            opcode, fin = b1 & 0x0F, b1 & 0x80
            if opcode == 0x8:
                await self._send(0x8, b'')
                self.closed = True
                return None
            if opcode == 0x9:
                await self._send(0xA, bytes(data))
                continue
            if opcode in (0x1, 0x2, 0x0):
                buf += bytes(data)
                if fin:
                    return buf.decode('utf-8', errors='replace')


class WebServer:
    def __init__(self, static_dir, routes, on_ws_message, on_ws_open=None):
        self.static_dir = static_dir
        self.routes = routes            # path -> callable(query) -> (status, ctype, bytes)
        self.on_ws_message = on_ws_message
        self.on_ws_open = on_ws_open
        self.clients = set()

    async def start(self, host, port):
        self.server = await asyncio.start_server(self._handle, host, port)
        return self.server

    async def broadcast(self, obj):
        if not self.clients:
            return
        text = json.dumps(obj, separators=(',', ':'))
        await asyncio.gather(*(c.send_text(text) for c in list(self.clients)), return_exceptions=True)

    async def _handle(self, reader, writer):
        try:
            request_line = await reader.readline()
            if not request_line:
                writer.close()
                return
            method, target, _ = request_line.decode('latin-1').split(' ', 2)
            headers = {}
            while True:
                line = await reader.readline()
                if line in (b'\r\n', b'\n', b''):
                    break
                k, _, v = line.decode('latin-1').partition(':')
                headers[k.strip().lower()] = v.strip()
            url = urlsplit(target)
            if url.path == '/ws' and headers.get('upgrade', '').lower() == 'websocket':
                await self._websocket(reader, writer, headers)
                return
            await self._http(writer, method, url)
        except Exception:
            try:
                writer.close()
            except Exception:
                pass

    async def _http(self, writer, method, url):
        path = url.path
        if path in self.routes:
            status, ctype, body = self.routes[path](parse_qs(url.query))
        else:
            rel = 'index.html' if path in ('/', '') else path.lstrip('/')
            full = os.path.normpath(os.path.join(self.static_dir, rel))
            if not full.startswith(os.path.normpath(self.static_dir)) or not os.path.isfile(full):
                status, ctype, body = 404, 'text/plain; charset=utf-8', b'No encontrado'
            else:
                ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
                if ctype.startswith('text/') or ctype in ('application/javascript',):
                    ctype += '; charset=utf-8'
                with open(full, 'rb') as f:
                    body = f.read()
                status = 200
        reason = {200: 'OK', 404: 'Not Found', 400: 'Bad Request'}.get(status, 'OK')
        head = (f'HTTP/1.1 {status} {reason}\r\nContent-Type: {ctype}\r\n'
                f'Content-Length: {len(body)}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n')
        writer.write(head.encode('latin-1') + (body if method != 'HEAD' else b''))
        await writer.drain()
        writer.close()

    async def _websocket(self, reader, writer, headers):
        key = headers.get('sec-websocket-key', '')
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                      f'Connection: Upgrade\r\nSec-WebSocket-Accept: {accept}\r\n\r\n').encode())
        await writer.drain()
        client = WSClient(reader, writer)
        self.clients.add(client)
        try:
            if self.on_ws_open:
                await self.on_ws_open(client)
            while not client.closed:
                msg = await client.recv()
                if msg is None:
                    break
                try:
                    await self.on_ws_message(client, json.loads(msg))
                except Exception as e:  # never let one bad message kill the socket
                    print(f'[websim] ws message error: {e!r}')
        finally:
            self.clients.discard(client)
            try:
                writer.close()
            except Exception:
                pass
