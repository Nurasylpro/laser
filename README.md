# WeedZap: Laser Weeding Robot

A robot that finds weeds with a camera and a custom YOLO model, then drives a laser over each one to burn it. No chemicals, no pulling.

▶️ **Final versions in action:** [YouTube](https://youtu.be/jQHW0xfkkLQ?si=JcptZxX9xMJJeqWb) · [Instagram @w_eedzap](https://www.instagram.com/w_eedzap?stkn=MWl5ODRhbG5hM2NrYg%3D%3D&utm_source=qr)

![The current WeedZap gantry with its camera, laser tool and drivers](docs/story/gantry-current.webp)

## The story

### 1. It started with a laser in our hands

Before any robot, we had a simple question: can light kill a weed? We took a laser, went out to the garden, and pointed it at a weed by hand.

| Treating the weed by hand | The same weed the next day |
|---|---|
| ![Pointing a hand-held laser at a weed in the garden](docs/story/hand-laser-test.webp) | ![The treated weed the next day, with damaged stems](docs/story/weed-next-day.webp) |

<a href="docs/story/first-hand-laser-test.mp4"><img src="docs/story/first-hand-laser-test.gif" width="260" alt="Video of the first hand-held laser test"></a>

🎥 [Watch the full video with sound](docs/story/first-hand-laser-test.mp4)

The next day, the weed showed the answer. The laser had destroyed its **phloem and xylem**, the channels that carry water and nutrients between the root and the leaves. Without them, the plant can't feed its upper part, so it dies. We don't need to burn the whole weed. One precise shot in the right place is enough. That became the core idea of WeedZap.

### 2. Version 1: a small robot for the field

Our first robot was a small wheeled rover built with my team. It drove along the field with a camera. A carriage on a rail moved the laser sideways: the camera detected a weed, and the carriage moved the laser to that position.

| The first field robot | Testing it together |
|---|---|
| ![The first WeedZap rover with camera and laser carriage on grass](docs/story/v1-field-robot.webp) | ![The team testing the rover outside](docs/story/v1-team-testing.webp) |

We worked on it for weeks: wiring, coding and testing outside until it worked. Then we presented WeedZap at an exhibition, with a solar-powered frame and a robotic arm version.

![WeedZap presented at the exhibition](docs/story/v1-expo.webp)

### 3. A real high-power laser: NUBM44 V2

The hand test showed the idea works, but a field robot needs real power. We moved to the **NUBM44 V2** high-power blue laser diode. We worked with its driver, connected it to a microcontroller and built a small web page to control its power over Wi-Fi.

![Controlling the NUBM44 V2 laser power from a web page](docs/story/nubm44-laser-control.webp)

### 4. The universal tool mount

For our smart greenhouse, we wanted the laser to work together with a mechanical tool. That meant designing a **universal tool mount**: one carriage that can pick up either the laser or a mechanical tool. This took a lot of design and many prints, more time than almost any other part.

| Design in CAD | Printed parts |
|---|---|
| ![CAD model of the universal tool mount](docs/story/tool-mount-cad.png) | ![The 3D-printed tool mount parts](docs/story/tool-mount-printed.webp) |

### 5. Teaching the robot to see weeds

A laser is useless if it doesn't know where to shoot, so we put a lot of work into computer vision. I collected many weeds myself, planted them in boxes, measured them and photographed them again and again. From those photos I built our own dataset, labelled it and trained **YOLO** models to find each weed and the exact point to hit.

| Growing and measuring weeds for the dataset | |
|---|---|
| ![Weeds planted in a box, measured with a ruler](docs/story/weeds-measured-1.jpg) | ![Another set of weeds measured for the dataset](docs/story/weeds-measured-2.jpg) |

| What the trained model sees | Detection from the gantry camera |
|---|---|
| ![Weed pose model detections with confidence scores](docs/images/weed-pose-model.jpg) | ![Detected weeds from the gantry camera](docs/images/gantry-detection.jpg) |

### 6. Path planning between weeds

Once every weed is found, the robot plans the shortest route between them, so the laser visits each weed once without wasting time.

<p align="center"><img src="docs/images/path-planning.jpg" width="100%" alt="Planned path between detected weeds"></p>

### 7. Today: the gantry

Everything came together in the current system: a gantry over the growing bed, a camera, the YOLO model, the NUBM44 laser and the tool mount. The rest of this README describes how it works.

| Zapping a weed in the soil bed | Testing late at night |
|---|---|
| ![The gantry firing the laser at a weed in the soil bed](docs/story/gantry-zapping-weed.webp) | ![The gantry running a laser test at night](docs/story/gantry-night-test.jpg) |

👉 For the final version of each part, watch the videos on [YouTube](https://youtu.be/jQHW0xfkkLQ?si=JcptZxX9xMJJeqWb) and [Instagram](https://www.instagram.com/w_eedzap?stkn=MWl5ODRhbG5hM2NrYg%3D%3D&utm_source=qr).

## Context

| | |
|---|---|
| **Type** | Team hardware project |
| **Built** | 2025 – 2026 |
| **Area** | Computer vision · robotics · embedded |

## How it works

1. **Tour.** The gantry sweeps both rows with no tool attached, so the camera can see. At each stop it photographs the bed and asks the firmware for its real position.
2. **Detect.** A YOLO pose model finds each weed and its crown (the point to burn).
3. **Map.** Pixel coordinates become millimetres using a per-height calibration (homography + polynomial fit), plus a measured camera-to-laser offset.
4. **Zap.** The gantry picks up the laser tool once, then visits every recorded weed and fires.

## Results (current gantry)

![Polynomial fit residuals at each calibration height](docs/images/calibration-residuals.png)

![Bench test of the model on printed weeds](docs/images/bench-test.jpg)

## Project structure

| Folder | What's inside |
|---|---|
| `scripts/` | `weed_zap.py` (full run), `path.py` (pixel → mm and routing), `calibrate.py`, `measure_laser_offset.py`, `test_model.py`, debugging tools |
| `model/` | Trained YOLO weights (`best.pt`, …) |
| `calibration/` | Saved calibration per height + camera-to-laser offset |
| `firmware/` | Arduino / ESP32 gantry firmware |

The branch `the-previous-version-without-cv-model` holds the earlier version, from before the CV model was added.

## Tech

Python · Ultralytics YOLO (pose) · OpenCV · NumPy · Arduino / ESP32 · Raspberry Pi

## What I learned

Training and deploying a detection model on real hardware, camera calibration, and debugging where a robot thinks it is versus where it really is.

## Follow WeedZap

- YouTube: https://youtu.be/jQHW0xfkkLQ?si=JcptZxX9xMJJeqWb
- Instagram: [@w_eedzap](https://www.instagram.com/w_eedzap?stkn=MWl5ODRhbG5hM2NrYg%3D%3D&utm_source=qr)
