"""Drawing helpers for the live tools."""
import cv2
import numpy as np

RED, GREEN, BLUE = (0, 0, 255), (0, 200, 0), (255, 80, 0)
YELLOW, WHITE, BLACK, CYAN = (0, 220, 255), (255, 255, 255), (0, 0, 0), (255, 255, 0)


def _pt(p):
    return int(round(p[0])), int(round(p[1]))


def _ok(p, shape, margin=4000):
    h, w = shape[:2]
    return np.all(np.isfinite(p)) and -margin < p[0] < w + margin and -margin < p[1] < h + margin


def line(img, a, b, color, thickness=2):
    if _ok(a, img.shape) and _ok(b, img.shape):
        cv2.line(img, _pt(a), _pt(b), color, thickness, cv2.LINE_AA)


def polyline(img, pix, color, closed=True, thickness=2):
    n = len(pix)
    for i in range(n if closed else n - 1):
        line(img, pix[i], pix[(i + 1) % n], color, thickness)


def axes(img, origin_px, x_px, y_px, z_px, thickness=3, labels=("X", "Y", "Z")):
    for p, c, lab in ((x_px, RED, labels[0]), (y_px, GREEN, labels[1]), (z_px, BLUE, labels[2])):
        line(img, origin_px, p, c, thickness)
        if _ok(p, img.shape):
            cv2.putText(img, lab, _pt(p), cv2.FONT_HERSHEY_SIMPLEX, 0.9, c, 2, cv2.LINE_AA)


def text_panel(img, lines, origin=(12, 12), scale=0.7, color=WHITE):
    """Lines of text on a translucent dark box, top-left."""
    if not lines:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    h = int(30 * scale / 0.7)
    w = max(cv2.getTextSize(t, font, scale, 2)[0][0] for t, *_ in _norm(lines)) + 20
    x0, y0 = origin
    roi = img[y0:y0 + h * len(lines) + 10, x0:x0 + w]
    roi[:] = (roi * 0.35).astype(img.dtype)
    for i, (t, c) in enumerate(_norm(lines, color)):
        cv2.putText(img, t, (x0 + 10, y0 + h * (i + 1)), font, scale, c, 2, cv2.LINE_AA)


def _norm(lines, color=WHITE):
    return [(l, color) if isinstance(l, str) else l for l in lines]


def coverage(img, cells, alpha=0.25):
    """Shade a grid of booleans (rows x cols) over the image: green = covered."""
    rows, cols = cells.shape
    h, w = img.shape[:2]
    over = img.copy()
    for r in range(rows):
        for c in range(cols):
            p0 = (int(c * w / cols), int(r * h / rows))
            p1 = (int((c + 1) * w / cols) - 1, int((r + 1) * h / rows) - 1)
            col = GREEN if cells[r, c] else RED
            cv2.rectangle(over, p0, p1, col, -1)
            cv2.rectangle(img, p0, p1, col, 1)
    cv2.addWeighted(over, alpha, img, 1 - alpha, 0, img)


def show(window, img, scale):
    disp = img if scale == 1 else cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.imshow(window, disp)
