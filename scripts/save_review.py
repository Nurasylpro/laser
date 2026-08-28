"""
save_review.py — annotate each tour stop's photo (weed crown + bounding box
+ computed absolute position) and save it to disk for offline review, no
blocking window. Lets you scroll through runs/review/ afterward and check
whether the position computed for each weed actually matches where it is
in the photo — the way to debug "fired who-knows-where."
"""

from pathlib import Path
import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
REVIEW_DIR   = PROJECT_ROOT / "runs" / "review"


def save_annotated(image_path: str, weeds: list, label: str) -> str:
    """
    Draw every weed's crown point, bounding box, and abs X/Y label onto a
    copy of the photo, save it to runs/review/<label>.jpg, return the path.

    weeds: list of dicts as produced by path.py's convert_weeds_to_mm —
    each has "crown_px"/"crown_py", optionally "box_xyxy", and "abs_x"/"abs_y".
    label: short filename-safe stem, e.g. "row1_stop3" — caller controls
    naming so files sort/group the way you want in the folder.
    """
    img = cv2.imread(image_path)
    if img is None:
        print(f"[WARN] save_review: cannot open image {image_path}, skipping.")
        return ""

    h, w = img.shape[:2]
    cx, cy = int(w / 2), int(h / 2)

    cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(img, "frame center", (cx + 8, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 0), 1, cv2.LINE_AA)

    for weed in weeds:
        px, py = int(weed["crown_px"]), int(weed["crown_py"])

        box = weed.get("box_xyxy")
        if box:
            x1, y1, x2, y2 = (int(v) for v in box)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # crown point — the exact pixel the pixel->mm conversion used
        cv2.drawMarker(img, (px, py), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)

        abs_x = weed.get("abs_x")
        abs_y = weed.get("abs_y")
        conf  = weed.get("conf")
        pos_text  = f"X={abs_x:+.1f} Y={abs_y:+.1f}mm" if abs_x is not None else "(out of range)"
        conf_text = f"conf={conf:.2f}" if conf is not None else ""
        cv2.putText(img, pos_text, (px + 10, py - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(img, conf_text, (px + 10, py + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 165, 255), 2, cv2.LINE_AA)

    bar_h = 30
    cv2.rectangle(img, (0, 0), (w, bar_h), (30, 30, 30), -1)
    cv2.putText(img, f"{label}  |  {len(weeds)} weed(s)", (8, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1, cv2.LINE_AA)

    REVIEW_DIR.mkdir(parents=True, exist_ok=True)
    out_path = REVIEW_DIR / f"{label}.jpg"
    cv2.imwrite(str(out_path), img)
    print(f"    [REVIEW] saved -> {out_path}")
    return str(out_path)
