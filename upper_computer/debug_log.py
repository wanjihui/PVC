"""Incremental decoding per sender, including CRLF and split UTF-8 code points."""
import codecs
from dataclasses import dataclass
from datetime import datetime
import time

@dataclass(frozen=True)
class LogLine:
    stamp: float
    source: str
    text: str

    def format(self, timestamp=True):
        prefix = datetime.fromtimestamp(self.stamp).strftime('%H:%M:%S.%f')[:-3]
        return (f'[{prefix}] ' if timestamp else '') + f'[{self.source}] {self.text}'

class LineDecoder:
    def __init__(self, encoding='utf-8', max_partial=8192, max_peers=32):
        self.encoding, self.max_partial, self.max_peers = encoding, max_partial, max_peers
        self.peers = {}

    def feed(self, packet):
        peer = packet.address
        result = []
        if peer not in self.peers:
            if len(self.peers) >= self.max_peers:
                result.extend(self.flush(min(self.peers, key=lambda x: self.peers[x]['seen'])))
            self.peers[peer] = dict(decoder=codecs.getincrementaldecoder(self.encoding)(errors='replace'),
                                    text='', stamp=packet.stamp, seen=time.monotonic())
        state = self.peers[peer]
        state['seen'] = time.monotonic()
        if not state['text'] and not state['decoder'].getstate()[0]:
            state['stamp'] = packet.stamp
        state['text'] += state['decoder'].decode(packet.data)
        while '\n' in state['text']:
            text, state['text'] = state['text'].split('\n', 1)
            result.append(LogLine(state['stamp'], f'{peer[0]}:{peer[1]}', text.rstrip('\r')))
            state['stamp'] = packet.stamp
        if len(state['text']) >= self.max_partial:
            result.append(LogLine(state['stamp'], f'{peer[0]}:{peer[1]}', state['text'] + ' [超长行已截断]'))
            state['text'] = ''
        return result

    def flush(self, peer=None):
        result = []
        for key in list(self.peers) if peer is None else [peer]:
            state = self.peers.pop(key)
            text = state['text'] + state['decoder'].decode(b'', final=True)
            if text:
                result.append(LogLine(state['stamp'], f'{key[0]}:{key[1]}', text.rstrip('\r')))
        return result

    def expire(self, idle_seconds=1):
        result = []
        for peer, state in list(self.peers.items()):
            if time.monotonic() - state['seen'] >= idle_seconds:
                result.extend(self.flush(peer))
        return result
