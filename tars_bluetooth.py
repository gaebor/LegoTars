"""Windows Bluetooth Classic transport for the Pybricks serial protocol."""
import asyncio
import ctypes as ct
from ctypes import wintypes as wt
import select
import socket
import struct
import sys

def encode(data):
    """LEGO COBS variant: escape 0/1/2, XOR 3, end with 2."""
    out = bytearray([255])
    index, count = 0, 1
    for byte in data:
        if byte > 2:
            out.append(byte)
            count += 1
        if byte <= 2 or count > 84:
            if byte <= 2:
                out[index] = byte * 84 + count + 2
            index = len(out)
            out.append(255)
            count = 1
    out[index] = count + 2
    return bytes(b ^ 3 for b in out) + b"\x02"


def decode(frame):
    if frame.startswith(b"\x01"):
        frame = frame[1:]
    data = bytes(b ^ 3 for b in frame)
    out = bytearray()
    pos = 0
    while pos < len(data):
        code = data[pos]
        pos += 1
        if code < 3:
            raise ValueError("Invalid COBS code")
        if code == 255:
            count, value = 84, None
        else:
            value, remainder = divmod(code - 3, 84)
            count = remainder
        if pos + count > len(data):
            raise ValueError("Truncated COBS block")
        out.extend(data[pos:pos + count])
        pos += count
        if pos < len(data) and value is not None:
            out.append(value)
    if not out:
        raise ValueError("Empty frame")
    return bytes(out)


def known_devices():
    """Read Windows' known classic devices; does not pair or change settings."""
    if sys.platform != "win32":
        raise RuntimeError("This Bluetooth adapter currently targets Windows")

    class Info(ct.Structure):
        _fields_ = [("size", wt.DWORD), ("address", ct.c_uint64),
                    ("device_class", wt.ULONG), ("connected", wt.BOOL),
                    ("remembered", wt.BOOL), ("authenticated", wt.BOOL),
                    ("last_seen", wt.WORD * 8), ("last_used", wt.WORD * 8),
                    ("name", wt.WCHAR * 248)]

    class Search(ct.Structure):
        _fields_ = [("size", wt.DWORD), ("authenticated", wt.BOOL),
                    ("remembered", wt.BOOL), ("unknown", wt.BOOL),
                    ("connected", wt.BOOL), ("inquiry", wt.BOOL),
                    ("timeout", ct.c_ubyte), ("radio", wt.HANDLE)]

    api = ct.WinDLL("BluetoothApis.dll", use_last_error=True)
    api.BluetoothFindFirstDevice.argtypes = [ct.POINTER(Search), ct.POINTER(Info)]
    api.BluetoothFindFirstDevice.restype = wt.HANDLE
    api.BluetoothFindNextDevice.argtypes = [wt.HANDLE, ct.POINTER(Info)]
    api.BluetoothFindNextDevice.restype = wt.BOOL
    api.BluetoothFindDeviceClose.argtypes = [wt.HANDLE]
    api.BluetoothFindDeviceClose.restype = wt.BOOL
    query = Search(ct.sizeof(Search), True, True, False, True, False, 0, None)
    info = Info()
    info.size = ct.sizeof(Info)
    handle = api.BluetoothFindFirstDevice(ct.byref(query), ct.byref(info))
    if not handle:
        error = ct.get_last_error()
        if error == 259:  # ERROR_NO_MORE_ITEMS
            return []
        raise ct.WinError(error)
    result = []
    try:
        while True:
            addr = f"{info.address:012X}"
            result.append({"name": info.name, "address": ":".join(addr[i:i+2] for i in range(0, 12, 2)),
                           "paired": bool(info.authenticated), "connected": bool(info.connected)})
            if not api.BluetoothFindNextDevice(handle, ct.byref(info)):
                error = ct.get_last_error()
                if error != 259:
                    raise ct.WinError(error)
                break
    finally:
        api.BluetoothFindDeviceClose(handle)
    return result


def open_socket(address, timeout=10):
    if sys.platform != "win32":
        raise RuntimeError("Bluetooth Classic adapter requires Windows")
    # ws2bth.h uses one-byte packing (SOCKADDR_BTH is 30 bytes).
    sockaddr = struct.pack("<HQ16sI", 32, int(address.replace(":", "").replace("-", ""), 16), bytes(16), 1)
    ws = ct.WinDLL("Ws2_32.dll")
    ws.connect.argtypes = [ct.c_size_t, ct.c_void_p, ct.c_int]
    ws.connect.restype = ct.c_int
    ws.WSAGetLastError.restype = ct.c_int
    sock = socket.socket(32, socket.SOCK_STREAM, 3)
    try:
        sock.setblocking(False)
        if ws.connect(sock.fileno(), sockaddr, len(sockaddr)) != 0:
            error = ws.WSAGetLastError()
            if error != 10035:  # WSAEWOULDBLOCK
                raise OSError(error, "Bluetooth RFCOMM connect failed")
            _, writable, exceptional = select.select([], [sock], [sock], timeout)
            if not writable and not exceptional:
                raise TimeoutError("Bluetooth connection timed out: power on, pair, and disconnect website")
            error = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
            if error:
                raise OSError(error, "Bluetooth RFCOMM connection failed")
        sock.settimeout(.2)
        return sock
    except BaseException:
        sock.close()
        raise


class BluetoothHub:
    def __init__(self, address):
        self.address = address
        self.sock = None
        self.reader = None
        self.frames = bytearray()
        self.stdout = bytearray()
        self.lines = asyncio.Queue()
        self.responses = asyncio.Queue()
        self.reads = asyncio.Queue()
        self.lock = asyncio.Lock()
        self.tag = 0
        self.max_write = 20
        self.info = {}

    async def connect(self):
        self.sock = await asyncio.to_thread(open_socket, self.address)
        self.reader = asyncio.create_task(self._reader())
        try:
            for key, char in (("name", 0x2A00), ("firmware", 0x2A26), ("protocol", 0x2A28)):
                self.info[key] = (await self.read_characteristic(1, char)).decode()
            if not self.info["protocol"].startswith("1.7."):
                raise RuntimeError("Expected Pybricks protocol 1.7.x, got " + self.info["protocol"])
            caps = await self.read_characteristic(2, 3)
            if len(caps) < 10:
                raise RuntimeError("Invalid hub capabilities")
            self.max_write = int.from_bytes(caps[:2], "little")
            if not 2 <= self.max_write <= 512:
                raise RuntimeError("Invalid maximum command size")
            await self._send(b"\x01\x01")  # subscribe (no response)
        except BaseException:
            await self.disconnect()
            raise

    async def _send(self, payload):
        await asyncio.to_thread(self.sock.sendall, encode(payload))

    def _dispatch(self, message):
        kind, payload = message[0], message[1:]
        if kind == 1:
            self.responses.put_nowait(payload)
        elif kind == 3:
            self.reads.put_nowait(payload)
        elif kind == 2 and payload and (payload[0] == 1 or payload[:2] == b"\x02\xff"):
            # stdout remains supported for firmware tracebacks; AppData carries agent JSON.
            self.stdout.extend(payload[2:] if payload[0] == 2 else payload[1:])
            if len(self.stdout) > 65536:
                raise RuntimeError("Agent output line exceeds limit")
            while b"\n" in self.stdout:
                line, _, rest = self.stdout.partition(b"\n")
                self.stdout = bytearray(rest)
                self.lines.put_nowait(line.rstrip(b"\r").decode("utf-8"))

    async def _reader(self):
        try:
            while True:
                try:
                    chunk = await asyncio.to_thread(self.sock.recv, 2048)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError("Bluetooth disconnected")
                self.frames.extend(chunk)
                while b"\x02" in self.frames:
                    frame, _, rest = self.frames.partition(b"\x02")
                    self.frames = bytearray(rest)
                    if frame:
                        self._dispatch(decode(frame))
                if len(self.frames) > 2048:
                    raise RuntimeError("Oversized Bluetooth frame")
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            for queue in (self.lines, self.responses, self.reads):
                queue.put_nowait(exc)

    async def _get(self, queue):
        item = await asyncio.wait_for(queue.get(), 5)
        if isinstance(item, Exception):
            raise item
        return item

    async def read_characteristic(self, service, char):
        async with self.lock:
            selector = struct.pack("<BH", service, char)
            await self._send(b"\x03" + selector)
            reply = await self._get(self.reads)
            if reply[:3] != selector:
                raise RuntimeError("Characteristic reply mismatch")
            return reply[3:]

    async def command(self, payload):
        async with self.lock:
            self.tag = (self.tag + 1) % 256
            await self._send(bytes([2, self.tag]) + payload)
            response = await self._get(self.responses)
            if len(response) != 5 or response[0] != self.tag:
                raise RuntimeError("Bluetooth command response mismatch")
            status = int.from_bytes(response[1:], "little")
            if status:
                raise RuntimeError(f"Pybricks command rejected: 0x{status:02x}")

    async def run(self, path=None, wait=False, print_output=False):
        if path is not None:
            raise ValueError("Deploy the agent with Pybricks Code")
        await self.command(b"\x01")

    async def stop_user_program(self):
        await self.command(b"\x00")

    async def write_line(self, line):
        data = (line + "\n").encode()
        for pos in range(0, len(data), self.max_write - 1):
            await self.command(b"\x06" + data[pos:pos + self.max_write - 1])

    async def read_line(self):
        # Client's timeout includes the whole operation, including local bursts.
        item = await self.lines.get()
        if isinstance(item, Exception):
            raise item
        return item

    async def disconnect(self):
        if self.sock:
            self.sock.close()
        if self.reader:
            self.reader.cancel()
            try:
                await self.reader
            except asyncio.CancelledError:
                pass
        self.sock = None
