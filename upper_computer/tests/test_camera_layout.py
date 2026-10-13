"""Check display sizing without altering camera pixels or their aspect ratio."""
import pytest

from app import camera_display_rect


@pytest.mark.parametrize('frame,canvas', [
    ((188, 120), (1500, 800)),
    ((188, 120), (800, 1000)),
    ((240, 400), (1100, 700)),
    ((640, 480), (160, 90)),
    ((320, 200), (640, 400)),
    ((188, 120), (1, 1)),
])
def test_fit_preserves_aspect_and_uses_available_space(frame, canvas):
    width, height, x, y = camera_display_rect(*frame, *canvas, fit=True, zoom=4)
    expected_scale = min(canvas[0] / frame[0], canvas[1] / frame[1])
    # One physical pixel of rounding is allowed; otherwise one dimension must fill.
    assert width == pytest.approx(frame[0] * expected_scale, abs=1)
    assert height == pytest.approx(frame[1] * expected_scale, abs=1)
    assert 0 < width <= canvas[0] + 1
    assert 0 < height <= canvas[1] + 1
    assert x == pytest.approx((canvas[0] - width) / 2, abs=1)
    assert y == pytest.approx((canvas[1] - height) / 2, abs=1)
    assert min(abs(width - canvas[0]), abs(height - canvas[1])) <= 1


@pytest.mark.parametrize('zoom', [1, 4, 8])
def test_manual_multiplier_changes_only_display_size(zoom):
    original = (188, 120)
    width, height, x, y = camera_display_rect(*original, 1600, 1000, fit=False, zoom=zoom)
    assert (width, height) == (188 * zoom, 120 * zoom)
    assert (x, y) == ((1600 - width) / 2, (1000 - height) / 2)
    assert original == (188, 120)


def test_manual_overflow_remains_accessible_from_origin():
    width, height, x, y = camera_display_rect(188, 120, 300, 200, fit=False, zoom=4)
    assert (width, height) == (752, 480)
    assert (x, y) == (0, 0)


def test_fit_ignores_previous_manual_multiplier():
    first = camera_display_rect(188, 120, 1000, 650, fit=True, zoom=1)
    second = camera_display_rect(188, 120, 1000, 650, fit=True, zoom=8)
    assert first == second
