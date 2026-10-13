"""MCAM v1: gray (1), row overlays (2), actual sampling rows/corners (3)."""
from dataclasses import dataclass
import struct
import time
import zlib

HEADER = struct.Struct('<4sBBHHIHHHI')
CHUNK_SIZE = 1400
MAX_PIXELS = 640 * 480
METADATA_SIZE = 12
OVERLAY_EDGE_ROWS = 20

@dataclass(frozen=True)
class Frame:
    width: int
    height: int
    frame_id: int
    pixels: bytes
    stamp: float
    address: tuple
    overlay: bytes = b''
    metadata: bytes = b''

    @property
    def sample_rows(self):
        return tuple(self.metadata[:2]) if self.metadata else None

    @property
    def corners(self):
        if not self.metadata:
            return ()
        return tuple((i, self.metadata[4+2*i], self.metadata[5+2*i])
                     for i in range(4) if self.metadata[2] & (1 << i))

def validate_metadata(metadata, width, height):
    if (len(metadata) != METADATA_SIZE or height > 255 or
        not 0 <= metadata[0] < metadata[1] < height or
        metadata[2] > 15 or metadata[3] > 3 or bool(metadata[2]) != bool(metadata[3])):
        raise ValueError('取值范围或拐点状态无效')
    for i in range(4):
        x, y = metadata[4+2*i:6+2*i]
        if metadata[2] & (1 << i):
            if x >= width or y >= height:
                raise ValueError('拐点坐标越界')
        elif (x, y) != (255, 255):
            raise ValueError('无效拐点必须标记为255')

def validate_overlay(overlay, width, height):
    if width > 255 or len(overlay) != 4 * height:
        raise ValueError('边界坐标尺寸无效')
    for y in range(height):
        left, right, center, flags = overlay[y*4:y*4+4]
        if flags > 3 or any(x != 255 and x >= width for x in (left, right, center)):
            raise ValueError('边界坐标或补线标记无效')

def encode_frame(pixels, width, height, frame_id, overlay=b'', metadata=b''):
    if not 0 < width <= 640 or not 0 < height <= 480 or len(pixels) != width * height:
        raise ValueError('灰度图尺寸与数据长度不一致')
    if overlay:
        validate_overlay(overlay, width, height)
    if metadata:
        if not overlay:
            raise ValueError('标注协议需要逐行边界数据')
        validate_metadata(metadata, width, height)
    payload = pixels + overlay + metadata
    count = (len(payload) + CHUNK_SIZE - 1) // CHUNK_SIZE
    crc = zlib.crc32(payload)
    fmt = 3 if metadata else (2 if overlay else 1)
    return [HEADER.pack(b'MCAM', 1, fmt, width, height, frame_id, i, count, len(chunk), crc) + chunk
            for i in range(count) for chunk in [payload[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE]]]

class CameraAssembler:
    def __init__(self, timeout=.8, max_pending=8):
        self.timeout, self.max_pending = timeout, max_pending
        self.pending = {}
        self.recent = {}
        self.completed = self.invalid = self.lost = self.duplicates = self.stale = 0

    def expire(self, now=None):
        now = time.monotonic() if now is None else now
        for key, state in list(self.pending.items()):
            if now - state['created'] > self.timeout:
                self.pending.pop(key)
                self.lost += 1
        self.recent = {k: v for k, v in self.recent.items() if now - v[1] < 2}

    def feed(self, packet, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        try:
            magic, version, fmt, w, h, fid, index, count, size, crc = HEADER.unpack_from(packet.data)
            total = w * h + (4 * h if fmt in (2, 3) else 0) + (METADATA_SIZE if fmt == 3 else 0)
            expected_count = (total + CHUNK_SIZE - 1) // CHUNK_SIZE
            expected_size = min(CHUNK_SIZE, total - index * CHUNK_SIZE)
            if (magic != b'MCAM' or version != 1 or fmt not in (1, 2, 3) or not 0 < w <= 640 or
                (fmt in (2, 3) and w > 255) or (fmt == 3 and h > 255) or
                not 0 < h <= 480 or not 0 < w * h <= MAX_PIXELS or count != expected_count or
                not 0 <= index < count or size != expected_size or len(packet.data) != HEADER.size + size):
                raise ValueError('invalid header')
        except (struct.error, ValueError):
            self.invalid += 1
            return None
        peer = packet.address
        old = self.recent.get(peer)
        if old and ((fid - old[0]) & 0xffffffff) >= 0x80000000:
            self.stale += 1
            return None
        if old and fid == old[0]:
            self.duplicates += 1
            return None
        key = (peer, fid)
        meta = (w, h, fmt, count, crc)
        state = self.pending.get(key)
        if state and state['meta'] != meta:
            self.pending.pop(key)
            self.invalid += 1
            return None
        if not state:
            if len(self.pending) >= self.max_pending:
                self.pending.pop(min(self.pending, key=lambda k: self.pending[k]['created']))
                self.lost += 1
            state = self.pending[key] = dict(meta=meta, chunks={}, created=now)
        payload = packet.data[HEADER.size:]
        if index in state['chunks']:
            self.duplicates += 1
            if state['chunks'][index] != payload:
                self.pending.pop(key)
                self.invalid += 1
            return None
        state['chunks'][index] = payload
        if len(state['chunks']) != count:
            return None
        pixels = b''.join(state['chunks'][i] for i in range(count))
        self.pending.pop(key)
        if zlib.crc32(pixels) != crc:
            self.invalid += 1
            return None
        overlay = pixels[w*h:w*h+4*h] if fmt in (2, 3) else b''
        metadata = pixels[w*h+4*h:] if fmt == 3 else b''
        if overlay:
            try:
                validate_overlay(overlay, w, h)
                if metadata:
                    validate_metadata(metadata, w, h)
            except ValueError:
                self.invalid += 1
                return None
        self.completed += 1
        self.recent[peer] = (fid, now)
        return Frame(w, h, fid, pixels[:w*h], packet.stamp, peer, overlay, metadata)

# Physical grayscale bytes remain intact; these colors are only a computer-side overlay.
LEFT_COLOR = (40, 180, 255)
RIGHT_COLOR = LEFT_COLOR
CENTER_COLOR = (255, 220, 70)
REPAIR_COLOR = (255, 60, 220)
SAMPLE_COLOR = (60, 255, 100)
CORNER_COLOR = (255, 70, 70)
CANDIDATE_COLOR = (255, 165, 60)

def compose_rgb(frame):
    rgb = bytearray(c for value in frame.pixels for c in (value, value, value))
    if not frame.overlay:
        return bytes(rgb)
    validate_overlay(frame.overlay, frame.width, frame.height)
    previous = [None] * 3
    # Only the overlay is cropped; all original grayscale rows remain visible.
    for y in range(OVERLAY_EDGE_ROWS, max(OVERLAY_EDGE_ROWS, frame.height-OVERLAY_EDGE_ROWS)):
        row = frame.overlay[4*y:4*y+4]
        for side, base in enumerate((LEFT_COLOR, RIGHT_COLOR, CENTER_COLOR)):
            x = row[side]
            if x == 255:
                previous[side] = None
                continue
            repair = side < 2 and bool(row[3] & (1 << side))
            color = REPAIR_COLOR if repair else base
            # Connect adjacent valid rows only. Never bridge missing rows.
            old = previous[side]
            if old is not None:
                ox, oy, was_repaired = old
                segment_color = REPAIR_COLOR if repair or was_repaired else base
                steps = max(abs(x-ox), y-oy)
                for n in range(steps+1):
                    px = round(ox + (x-ox)*n/steps)
                    py = round(oy + (y-oy)*n/steps)
                    offset = 3 * (py*frame.width+px)
                    rgb[offset:offset+3] = bytes(segment_color)
            offset = 3 * (y*frame.width+x)
            rgb[offset:offset+3] = bytes(color)
            previous[side] = (x, y, repair)
    if frame.metadata:
        validate_metadata(frame.metadata, frame.width, frame.height)
        # Full image-width green guides, including when the live preview row changes.
        for y in frame.sample_rows:
            start = 3 * y * frame.width
            rgb[start:start+3*frame.width] = bytes(SAMPLE_COLOR) * frame.width
        # Cross markers are drawn last so corners on a guide remain identifiable.
        for _, x, y in frame.corners:
            if not OVERLAY_EDGE_ROWS <= y < frame.height-OVERLAY_EDGE_ROWS:
                continue
            for delta in range(-3, 4):
                for px, py in ((x+delta, y), (x, y+delta)):
                    if 0 <= px < frame.width and OVERLAY_EDGE_ROWS <= py < frame.height-OVERLAY_EDGE_ROWS:
                        offset = 3 * (py*frame.width+px)
                        rgb[offset:offset+3] = bytes(CANDIDATE_COLOR if frame.metadata[3] == 1 else CORNER_COLOR)
    return bytes(rgb)

def frame_rgba(frame):
    rgb = compose_rgb(frame)
    return [c for i in range(0, len(rgb), 3)
            for c in (rgb[i]/255, rgb[i+1]/255, rgb[i+2]/255, 1.0)]

def frame_bmp(frame):
    """Save the displayed gray/overlay image as a standard 24-bit BMP."""
    if not frame.overlay:
        # Preserve the original 8-bit gray export for old camera senders.
        stride = (frame.width+3) & ~3
        image = b''.join(frame.pixels[y*frame.width:(y+1)*frame.width] + bytes(stride-frame.width)
                         for y in range(frame.height-1, -1, -1))
        palette = b''.join(bytes((i, i, i, 0)) for i in range(256))
        return (struct.pack('<2sIHHI', b'BM', 1078+len(image), 0, 0, 1078)
                + struct.pack('<IiiHHIIiiII', 40, frame.width, frame.height, 1, 8, 0,
                              len(image), 2835, 2835, 256, 256) + palette + image)
    rgb = compose_rgb(frame)
    stride = (3*frame.width+3) & ~3
    rows = []
    for y in range(frame.height-1, -1, -1):
        row = rgb[y*frame.width*3:(y+1)*frame.width*3]
        rows.append(b''.join(row[i:i+3][::-1] for i in range(0, len(row), 3))
                    + bytes(stride-len(row)))
    image = b''.join(rows)
    header = struct.pack('<2sIHHI', b'BM', 54+len(image), 0, 0, 54)
    dib = struct.pack('<IiiHHIIiiII', 40, frame.width, frame.height, 1, 24, 0,
                      len(image), 2835, 2835, 0, 0)
    return header + dib + image
