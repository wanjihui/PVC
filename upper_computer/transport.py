"""Bounded, thread-based UDP receivers. GUI work is always done on the main thread."""
from dataclasses import dataclass
import queue
import socket
import threading
import time
from multiplex_protocol import decode_packet

@dataclass(frozen=True)
class Packet:
    stamp: float
    address: tuple
    data: bytes

class UdpReceiver:
    def __init__(self, capacity=4096):
        self.queue = queue.Queue(capacity)
        self.received = self.dropped = 0
        self.error = ''
        self._socket = self._thread = None
        self._stop = threading.Event()

    def start(self, ip, port):
        if self._socket is not None:
            raise RuntimeError('端口已经启动')
        if not 1 <= port <= 65535:
            raise ValueError('监听端口必须为 1～65535')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Windows exclusive binding prevents another monitor silently stealing packets.
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            sock.bind((ip, port))
            sock.settimeout(.2)
        except Exception:
            sock.close()
            raise
        self.received = self.dropped = 0
        self.error = ''
        self._stop.clear()
        self._socket = sock
        self._thread = threading.Thread(target=self._run, args=(sock,), daemon=True)
        self._thread.start()

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def _run(self, sock):
        while not self._stop.is_set():
            try:
                data, address = sock.recvfrom(65535)
                item = Packet(time.time(), address, data)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self.error = str(exc)
                break
            self.received += 1
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                self.dropped += 1

    def stop(self):
        self._stop.set()
        if self._socket:
            self._socket.close()
        if self._thread:
            self._thread.join(timeout=1)
        self._socket = self._thread = None

    def drain(self, limit=256):
        for _ in range(limit):
            try:
                yield self.queue.get_nowait()
            except queue.Empty:
                break

def send_udp(data, ip, port):
    if not 1 <= port <= 65535:
        raise ValueError('目标端口必须为 1～65535')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        return sock.sendto(data, (ip, port))


class RoutedReceiver:
    """A display channel queue; it owns no socket and follows the shared listener."""
    def __init__(self, owner, capacity):
        self.owner = owner
        self.queue = queue.Queue(capacity)
        self.received = self.dropped = 0

    @property
    def running(self):
        return self.owner.receiver.running

    @property
    def error(self):
        return self.owner.receiver.error

    def stop(self):
        self.owner.stop()

    def drain(self, limit=256):
        for _ in range(limit):
            try:
                yield self.queue.get_nowait()
            except queue.Empty:
                break


class MultiplexReceiver:
    """One exclusive UDP listener routes validated frames into three bounded queues."""
    def __init__(self, capacity=4096):
        self.receiver = UdpReceiver(capacity)
        self.channels = {name:RoutedReceiver(self,capacity) for name in ('data','log','camera')}
        self.invalid = 0
        self.address = None

    def start(self, ip, port):
        self.receiver.start(ip,port)
        self.address = (ip,port)
        self.invalid = 0
        for channel in self.channels.values():
            channel.received = channel.dropped = 0

    def route(self, limit=4096):
        for packet in self.receiver.drain(limit):
            try:
                name,payload = decode_packet(packet.data)
            except ValueError:
                self.invalid += 1
                continue
            channel = self.channels[name]
            channel.received += 1
            try:
                channel.queue.put_nowait(Packet(packet.stamp,packet.address,payload))
            except queue.Full:
                channel.dropped += 1

    def stop(self):
        self.receiver.stop()
