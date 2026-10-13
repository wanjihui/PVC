"""Receive-gated UDP heartbeat. It never changes vehicle motion or gains."""
class LinkHealth:
    def __init__(self):
        self.peer = None
        self.last_rx = None
        self.last_send = self.last_report = -1e9
        self.sequence = self.sent = self.send_errors = 0

    def received(self, peer, now):
        if self.peer is None:
            self.peer = peer
        if peer == self.peer:
            self.last_rx = now

    def heartbeat(self, now, enabled=True):
        # No reply once all incoming vehicle traffic has been absent for 4s.
        # It proves recent reception, rather than blindly keeping a dead link healthy.
        if (not enabled or self.peer is None or self.last_rx is None or
                now-self.last_rx > 4 or now-self.last_send < 1):
            return None
        self.last_send = now
        payload = f'MCAR ALIVE {self.sequence}\n'.encode('ascii')
        self.sequence = (self.sequence+1) & 0x7fffffff
        return payload, self.peer
