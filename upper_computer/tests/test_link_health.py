from link_health import LinkHealth

def test_only_reply_to_recent_known_vehicle_and_rate_limit():
    h=LinkHealth();peer=('127.0.0.1',6666)
    assert h.heartbeat(0) is None
    h.received(peer,0)
    assert h.heartbeat(0)==(b'MCAR ALIVE 0\n',peer)
    assert h.heartbeat(.9) is None
    assert h.heartbeat(1)==(b'MCAR ALIVE 1\n',peer)
    h.received(('127.0.0.2',6666),3)
    assert h.heartbeat(4.1) is None # Foreign traffic cannot conceal outage.
    h.received(peer,10)
    assert h.heartbeat(10,False) is None
    assert h.heartbeat(10)[0]==b'MCAR ALIVE 2\n'

def test_sequence_wrap():
    h=LinkHealth();h.received(('127.0.0.1',6666),0);h.sequence=0x7fffffff
    assert h.heartbeat(0)[0]==b'MCAR ALIVE 2147483647\n'
    assert h.heartbeat(1)[0]==b'MCAR ALIVE 0\n'
