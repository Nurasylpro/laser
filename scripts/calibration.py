import cv2
import numpy as np
import os
# ========= SETTINGS =========
IMAGE_PATH = r"data\raw\calibration2\165.jpg"  # put your image path here
A4_W_MM =380.0
A4_H_MM = 200.0

# Order must be:
# 1) top-left
# 2) top-right
# 3) bottom-right
# 4) bottom-left
clicked_points = []

def mouse_callback(event, x, y, flags, param):
    global clicked_points, img_show

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_points) < 4:
            clicked_points.append([x, y])
            cv2.circle(img_show, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(
                img_show,
                str(len(clicked_points)),
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )
            cv2.imshow("Click A4 corners", img_show)

# Load image
img = cv2.imread(IMAGE_PATH)
if img is None:
    raise FileNotFoundError(f"Cannot open image: {IMAGE_PATH}")

img_show = img.copy()

cv2.namedWindow("Click A4 corners", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Click A4 corners", mouse_callback)

print("Click 4 A4 corners in this order:")
print("1) top-left")
print("2) top-right")
print("3) bottom-right")
print("4) bottom-left")

while True:
    cv2.imshow("Click A4 corners", img_show)
    key = cv2.waitKey(1) & 0xFF

    # press r to reset
    if key == ord('r'):
        clicked_points = []
        img_show = img.copy()
        print("Reset.")

    # press q to finish after 4 points
    if key == ord('q'):
        if len(clicked_points) == 4:
            break
        else:
            print("You must click exactly 4 points first.")

cv2.destroyAllWindows()

img_pts = np.array(clicked_points, dtype=np.float32)

# Real-world coordinates of A4 corners in millimeters
world_pts = np.array([
    [0, 0],                 # top-left
    [A4_W_MM, 0],           # top-right
    [A4_W_MM, A4_H_MM],     # bottom-right
    [0, A4_H_MM]            # bottom-left
], dtype=np.float32)

# Homography: image pixels -> millimeters
H = cv2.getPerspectiveTransform(img_pts, world_pts)

print("\nHomography matrix (pixel -> mm):")
print(H)
 


os.makedirs("calibration", exist_ok=True)

np.savez(
    "C:/Users/nuamy/Desktop/dauren/calibration2/z_165.npz",
    H=H,
    img_pts=img_pts,
    world_pts=world_pts,
    a4_width_mm=383.0,
    a4_height_mm=200.0
)

print("Saved calibration to calibration/stage1_homography_a4_centered.npz")
def pixel_to_mm(x_px, y_px, H):
    pt = np.array([[[x_px, y_px]]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(pt, H)
    x_mm, y_mm = mapped[0][0]
    return float(x_mm), float(y_mm)



# Optional: create a top-down warped preview
preview_scale = 3  # pixels per mm for visualization
dst_w = int(A4_W_MM * preview_scale)
dst_h = int(A4_H_MM * preview_scale)

preview_pts = np.array([
    [0, 0],
    [dst_w, 0],
    [dst_w, dst_h],
    [0, dst_h]
], dtype=np.float32)

H_preview = cv2.getPerspectiveTransform(img_pts, preview_pts)
warped = cv2.warpPerspective(img, H_preview, (dst_w, dst_h))

