/*
  WeedZap — Direction + Limit Switch Test
  Arduino IDE (NOT PlatformIO)

  Forked into laser/ 2026-08-03 from dauren/firmware/test_sketch/test_sketch.ino
  (the reference design). Only change from that copy: printPositions() below
  was rewritten to print a clean "X=<mm>mm Y=<mm>mm Z=<mm>mm" line (see its
  comment) — the old "POS XL=<steps>(<mm>mm) XR=... Y=... Z=..." format could
  never be parsed by the Python side's get_gantry_position(), which looks for
  a literal "X=" substring immediately followed by a number ("XL=" doesn't
  match "X="), so every 'p' query would hang forever. Everything else
  (homing, soft limits, absolute mm interface, tool take/put, weeding pass,
  laser/servo shared-pin handling) is unchanged from the dauren copy.

  Pins match current main.cpp wiring:
    Z  : PUL=2   DIR=A0
    Y  : PUL=4   DIR=12
    XR : PUL=6   DIR=7
    XL : PUL=8   DIR=9

  Switches (INPUT_PULLUP), trigger levels measured empirically 2026-07-27:
    XR=A5  XL=A4  Y=A2   -> rest=LOW,  triggered=HIGH
    Z =A3                -> rest=HIGH, triggered=LOW   (opposite of the other three)

  LED removed entirely 2026-07-31 — hardware taken off the machine.

  Serial commands (115200 baud):
    sw               — print switch states once
    pos / p          — print current position (X=<mm>mm Y=<mm>mm Z=<mm>mm)
    z  <mm>          — move Z  to <mm> away from home (0)      <- requires homed
    y  <mm>          — move Y  to <mm> away from home (0)      <- requires homed
    xr <mm>          — move XR to <mm> away from home (0)      <- requires homed
    xl <mm>          — move XL to <mm> away from home (0)      <- requires homed
    x  <mm>          — move XL+XR together to <mm> away from home, as one unit
    xyz <x> <y> <z>  — move all 3 axes at once. "m <x> <y> <z>" is the same
    m   <x> <y> <z>    command — kept as an alias since the Python control
                        scripts already send "m". BLOCKING: each axis moves
                        at its own full speed (so Z, the shortest travel,
                        still clears out of the way fast internally), but
                        the command does not return/print until every axis
                        has actually arrived. Prints "Move complete" on
                        success — requires homed.
    h                — home all axes sequentially: Z, then Y, then X. One
                        axis at a time, no simultaneous moves. Prints
                        "HOMING COMPLETE (HOMED)" on success.
    tool take 1/2    — pick up tool 1 or 2 from its rack (see Tool change below)
    tool put         — return the currently held tool to its rack
    weed             — run a weeding pass with tool 2 (see Weeding pass below)
    servo <-5..72>   — manual blade servo test, offset from SERVO_CENTER_DEG
                        (90) — real angle = 90+offset. 72 = blades open
                        (start), -5 = blades closed (measured 2026-07-31 on
                        the new servo). Speed-controlled (steps 1deg at a
                        time, see BLADE_STEP_DELAY_MS, currently slow on
                        purpose) — the "weed" pass uses an instant jump
                        instead so the gantry doesn't stall.
    laser <0-255>    — laser power (PWM), same pin as the servo — detaches
                        the servo first if it's attached (see below)
    stop             — stop all motors, zero their position counters (also
                        opens the blade and kills the laser)

  PYTHON COMPATIBILITY — "m" (alias for "xyz"), "p" (alias for "pos"), and
  the "HOMING COMPLETE" wording all exist ONLY because the Python control
  scripts (scripts/weed_zap.py, scripts/move.py) already send/expect exactly
  these strings. Do not remove them without checking those scripts again.
  Also: "xyz"/"m" is blocking (via blockMoveXYZ()) — the per-axis
  independent-speed behavior (Z clears first) is unchanged, only the moment
  the serial response is sent is delayed until every axis has truly arrived,
  so Python can safely fire the laser right after "Move complete".

  SHARED PIN (10) — laser TTL (tool 1) and blade servo (tool 2) use the same
  physical wire, since the two tools are never mounted at once. The servo
  library hijacks Timer1 while attached, which breaks plain PWM (analogWrite)
  on that pin — so "laser <val>" always calls bladeServo.detach() first, and
  the servo is only ever attach()'d while tool 2 is actually held or being
  manually tested (see toolTake/toolPut and the "servo" command).

  All movement commands are ABSOLUTE distance-from-home in millimeters, not
  relative jogs and not raw steps — "z 50" always means "go to 50mm from
  home," no matter where Z currently is. Requires homing first (h) since
  distance-from-home is meaningless before that. Values are clamped to each
  axis's real travel range (see Soft limits below) — over-range requests get
  silently reduced to the max, with a "SOFT LIMIT" message.

  Switch states are NOT printed automatically — call "sw" whenever you want
  to check them.

  TOOL CHANGE — racks arranged along X, approach/retreat happens on X via
  TOOL_X_OFFSET.
    TAKE: coarse xyz move to (X+OFFSET, Y, Z), then X-only move to true X
          (arrival = engaged), then Z -> Z1 (lifts tool clear).
    PUT:  coarse xyz move to (X+OFFSET, Y, Z1), then X-only move to true X
          (still at clearance height), then Z -> Z (seats tool on rack),
          then X-only move back to X+OFFSET (releases magnet).
  Only the coarse first leg moves multiple axes at once — the actual
  engage/disengage motion right next to the magnet is always a single X-only
  move, never combined with Y or Z, to avoid side-loading the coupling.

  WEEDING PASS — "weed" command, tool 2 only, assumes "tool take 2" already
  ran. X is the sweep (full bed width); Y just selects which row. Sweeps
  run far-end-to-home (X_END -> X_START), reversed 2026-08-06 from the
  original home-to-far-end direction:
    1. Row 1: X repositions to WEED_X_END_MM FIRST (no cutting yet —
       blade open), then Y -> WEED_ROW1_Y_MM. Z descends to WEED_Z_MM
       (soil depth) — safe here since X_END is well clear of the tool
       rack, same as X_START was. Blade closes (slow). X sweeps back to
       WEED_X_START_MM at WEED_RUN_SPEED_X, with the blade continuously cycling
       closed->open->closed once every WEED_CYCLE_MM (90mm) of X travel —
       the target angle is computed from X's live position every loop
       iteration (a triangle wave over DISTANCE, not time), so it can
       never drift out of sync with X regardless of actual speed. Blade
       opens (slow) once X stops, Z raises to WEED_RETREAT_Z_MM (clear of
       the soil). X is now already at X_START — no extra retreat move.
    2. Row 2: same, at WEED_ROW2_Y_MM.

  MEASURED RESULTS (2026-07-27):
    Z : negative moves toward switch
    Y : positive moves toward switch
    XR: negative moves toward switch
    XL: positive moves toward switch
  Since XL and XR need opposite signs to move toward their own switches,
  they are mirror-mounted. The "x" command applies +steps to XL and
  -steps to XR so a single positive value drives the whole gantry toward
  the switches without racking.
*/

#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// ── Pin definitions (match main.cpp) ───────────────────────────────
#define Z_PUL   2
#define Z_DIR   A0
#define Y_PUL   4
#define Y_DIR   12
#define XR_PUL  6
#define XR_DIR  7
#define XL_PUL  8
#define XL_DIR  9

#define SW_XR  A5
#define SW_XL  A4
#define SW_Z   A3
#define SW_Y   A2

// Trigger levels, measured empirically 2026-07-27 (opposite of original
// assumption for ALL four switches):
#define SW_TRIG_XR  HIGH   // rest=LOW, pressed=HIGH
#define SW_TRIG_XL  HIGH   // rest=LOW, pressed=HIGH
#define SW_TRIG_Y   HIGH   // rest=LOW, pressed=HIGH
#define SW_TRIG_Z   LOW    // rest=HIGH, pressed=LOW

// ── Homing config (confirmed 2026-07-27) ───────────────────────────
// dir applies directly to Z/Y/XL; XR always takes the opposite sign of XL.
#define HOME_DIR_Z   -1
#define HOME_DIR_Y   +1
#define HOME_DIR_X   +1
#define HOME_SPEED    3200
#define HOME_ACCEL    800

// Final pull-off after the switch is hit, in steps — this defines where
// "home" (position 0) actually sits, offset away from the physical switch.
#define HOME_BACKOFF_Z  800    // 800 steps = 1.0mm  @ 800 steps/mm
#define HOME_BACKOFF_Y  800    // 800 steps = 1.0mm  @ 800 steps/mm
#define HOME_BACKOFF_X  80     // 80  steps = 2.0mm  @  40 steps/mm

// Max search distance per axis, in steps — homing gives up (FAIL) once this
// many steps have been commanded without the switch triggering.
#define HOME_MAX_STEPS_Z  176000L    // 200mm * 800 steps/mm * 1.1
#define HOME_MAX_STEPS_Y  440000L    // 500mm * 800 steps/mm * 1.1
#define HOME_MAX_STEPS_X  35000L     // 800mm *  40 steps/mm * 1.1 — applies to XL and XR individually
#define HOME_ALREADY_TRIGGERED_BACKOFF  4000L  // used only if switch is hit at start of homing

// Per-axis run speed/accel — kept separate because Z/Y (lead screw, 800
// steps/mm) and X (belt, 40 steps/mm) want very different tuning even at
// the same physical mm/s.
#define RUN_SPEED_Z  3200   // steps/sec — Uno + AccelStepper reliable ceiling is ~4000
#define RUN_ACCEL_Z  800
#define RUN_SPEED_Y  3200
#define RUN_ACCEL_Y  800
#define RUN_SPEED_X  3200
#define RUN_ACCEL_X  800

// ── Soft limits ─────────────────────────────────────────────────
#define Z_TRAVEL_STEPS   140000L   // 175mm * 800 steps/mm
#define Y_TRAVEL_STEPS  400000L    // 500mm * 800 steps/mm
#define X_TRAVEL_STEPS   32000L    // 800mm *  40 steps/mm (per XL/XR)
// Operating ranges, in each motor's own position coordinate:
//   Z : [0, +Z_TRAVEL_STEPS]   (0 = home/switch side, + = away)
//   Y : [-Y_TRAVEL_STEPS, 0]   (0 = home/switch side, - = away)
//   XL: [-X_TRAVEL_STEPS, 0]   (0 = home/switch side, - = away)
//   XR: [0, +X_TRAVEL_STEPS]   (0 = home/switch side, + = away)

// ── mm <-> steps ────────────────────────────────────────────────
#define STEPS_PER_MM_Z  800
#define STEPS_PER_MM_Y  800
#define STEPS_PER_MM_X   40   // belt drive, applies to XL and XR

// ── Tool change (magnetic coupling) ────────────────────────────
#define TOOL1_X   10  // laser
#define TOOL1_Y   330
#define TOOL1_Z   27
#define TOOL1_Z1  0     // full raise to home rather than partial
#define TOOL2_X   19   // mech tool / servo
#define TOOL2_Y   180
#define TOOL2_Z   44
#define TOOL2_Z1  0
#define TOOL_X_OFFSET  100   // mm, e.g. tool2: X 19 -> approach/retreat at 119
// Tool 1 (laser) gets 10mm more approach/retreat clearance than
// TOOL_X_OFFSET alone gives it — 2026-08-04, user found the coarse
// xyz move from the tool-1 rack toward row 1 was passing too close to
// the mechanical tool's rack (TOOL2_X=19, TOOL2_Y=180). Only tool 1 is
// widened; tool 2 keeps the shared TOOL_X_OFFSET.
#define TOOL1_X_OFFSET  (TOOL_X_OFFSET + 10)   // = 110, approach/retreat at 10+110=120

// ── Mechanical weeder blade (tool 2 only) ──────────────────────
#define SERVO_PIN          10
#define SERVO_CENTER_DEG   90
#define BLADE_OPEN_DEG      72   // blades open/separate — starting position
#define BLADE_CLOSED_DEG    -5   // blades closed (cutting)

// Laser (tool 1) — same physical pin as the servo.
#define LASER_PIN  10

// Weeding pass (tool 2), assumes "tool take 2" already ran.
#define WEED_Z_MM         155
#define WEED_RETREAT_Z_MM 100
#define WEED_ROW1_Y_MM    120
#define WEED_ROW2_Y_MM    330
#define WEED_X_START_MM  (TOOL2_X + TOOL_X_OFFSET)
#define WEED_X_END_MM     680
#define WEED_CYCLE_MM      90
#define WEED_RUN_SPEED_X  3200
#define WEED_RUN_ACCEL_X   800

// ── Steppers ────────────────────────────────────────────────────
Servo bladeServo;
AccelStepper motorZ (AccelStepper::DRIVER, Z_PUL,  Z_DIR);
AccelStepper motorY (AccelStepper::DRIVER, Y_PUL,  Y_DIR);
AccelStepper motorXR(AccelStepper::DRIVER, XR_PUL, XR_DIR);
AccelStepper motorXL(AccelStepper::DRIVER, XL_PUL, XL_DIR);

// ── Globals ──────────────────────────────────────────────────────
String inputBuffer = "";
bool   isHomed     = false;
int    currentTool = 0;   // 0 = none, 1/2 = tool held
int    bladeCurrentOffset = 0;

// ── Weeder blade (servo) ────────────────────────────────────────
void bladeWrite(int offsetDeg) {
  offsetDeg = constrain(offsetDeg, BLADE_CLOSED_DEG, BLADE_OPEN_DEG);
  bladeServo.write(constrain(SERVO_CENTER_DEG + offsetDeg, 0, 180));
  bladeCurrentOffset = offsetDeg;
}
void bladeOpen()  { bladeWrite(BLADE_OPEN_DEG); }
void bladeClose() { bladeWrite(BLADE_CLOSED_DEG); }

#define BLADE_STEP_DELAY_MS  60   // ms per degree — ~16.7 deg/sec
void bladeMoveTo(int targetOffset) {
  targetOffset = constrain(targetOffset, BLADE_CLOSED_DEG, BLADE_OPEN_DEG);
  int step = (targetOffset > bladeCurrentOffset) ? 1 : -1;
  while (bladeCurrentOffset != targetOffset) {
    bladeWrite(bladeCurrentOffset + step);
    delay(BLADE_STEP_DELAY_MS);
  }
}

// ── Helpers ──────────────────────────────────────────────────────
void printSwitches() {
  Serial.print(F("SW raw  XR=")); Serial.print(digitalRead(SW_XR) == HIGH ? "HIGH" : "LOW");
  Serial.print(F("  XL="));       Serial.print(digitalRead(SW_XL) == HIGH ? "HIGH" : "LOW");
  Serial.print(F("  Y="));        Serial.print(digitalRead(SW_Y)  == HIGH ? "HIGH" : "LOW");
  Serial.print(F("  Z="));        Serial.println(digitalRead(SW_Z) == HIGH ? "HIGH" : "LOW");
}

// Rewritten 2026-08-03 (laser/ fork): the old format
// "POS XL=<steps>(<mm>mm) XR=... Y=... Z=..." was never parseable by
// get_gantry_position() on the Python side (it looks for a plain "X="
// token). Now prints exactly "X=<mm>mm Y=<mm>mm Z=<mm>mm" — X is XL's
// mm value (XR is always its mirror, same magnitude, no separate field
// needed for position tracking).
void printPositions() {
  float xmm = fabs(motorXL.currentPosition()) / (float)STEPS_PER_MM_X;
  float ymm = fabs(motorY.currentPosition())  / (float)STEPS_PER_MM_Y;
  float zmm = fabs(motorZ.currentPosition())  / (float)STEPS_PER_MM_Z;
  Serial.print(F("X=")); Serial.print(xmm); Serial.print(F("mm "));
  Serial.print(F("Y="));  Serial.print(ymm); Serial.print(F("mm "));
  Serial.print(F("Z="));  Serial.print(zmm); Serial.println(F("mm"));
}

// Clamps an absolute target position to [minPos, maxPos].
long clampAbs(long target, long minPos, long maxPos) {
  if (target > maxPos) target = maxPos;
  if (target < minPos) target = minPos;
  return target;
}

void blockMoveZ(float mm) {
  long target = clampAbs(lroundf(mm * STEPS_PER_MM_Z), 0, Z_TRAVEL_STEPS);
  motorZ.setMaxSpeed(RUN_SPEED_Z); motorZ.setAcceleration(RUN_ACCEL_Z);
  motorZ.moveTo(target);
  while (motorZ.run()) {}
}

void blockMoveY(float mm) {
  long target = clampAbs(-lroundf(mm * STEPS_PER_MM_Y), -Y_TRAVEL_STEPS, 0);
  motorY.setMaxSpeed(RUN_SPEED_Y); motorY.setAcceleration(RUN_ACCEL_Y);
  motorY.moveTo(target);
  while (motorY.run()) {}
}

void blockMoveX(float mm) { // XL+XR together
  long targetXL = clampAbs(-lroundf(mm * STEPS_PER_MM_X), -X_TRAVEL_STEPS, 0);
  long targetXR = clampAbs( lroundf(mm * STEPS_PER_MM_X),  0, X_TRAVEL_STEPS);
  motorXL.setMaxSpeed(RUN_SPEED_X); motorXL.setAcceleration(RUN_ACCEL_X);
  motorXR.setMaxSpeed(RUN_SPEED_X); motorXR.setAcceleration(RUN_ACCEL_X);
  motorXL.moveTo(targetXL);
  motorXR.moveTo(targetXR);
  // NOTE: must NOT write "while (motorXL.run() || motorXR.run())" — || short-
  // circuits, so motorXR.run() would never be called while XL still had
  // distance to go, running them sequentially instead of together.
  while (motorXL.distanceToGo() != 0 || motorXR.distanceToGo() != 0) {
    motorXL.run();
    motorXR.run();
  }
}

void blockMoveXYZ(float xmm, float ymm, float zmm) {
  long targetZ  = clampAbs(lroundf(zmm * STEPS_PER_MM_Z),  0, Z_TRAVEL_STEPS);
  long targetY  = clampAbs(-lroundf(ymm * STEPS_PER_MM_Y), -Y_TRAVEL_STEPS, 0);
  long targetXL = clampAbs(-lroundf(xmm * STEPS_PER_MM_X), -X_TRAVEL_STEPS, 0);
  long targetXR = clampAbs( lroundf(xmm * STEPS_PER_MM_X),  0, X_TRAVEL_STEPS);
  motorZ.setMaxSpeed(RUN_SPEED_Z);   motorZ.setAcceleration(RUN_ACCEL_Z);
  motorY.setMaxSpeed(RUN_SPEED_Y);   motorY.setAcceleration(RUN_ACCEL_Y);
  motorXL.setMaxSpeed(RUN_SPEED_X);  motorXL.setAcceleration(RUN_ACCEL_X);
  motorXR.setMaxSpeed(RUN_SPEED_X);  motorXR.setAcceleration(RUN_ACCEL_X);
  motorZ.moveTo(targetZ);
  motorY.moveTo(targetY);
  motorXL.moveTo(targetXL);
  motorXR.moveTo(targetXR);
  while (motorZ.distanceToGo() != 0 || motorY.distanceToGo() != 0 ||
         motorXL.distanceToGo() != 0 || motorXR.distanceToGo() != 0) {
    motorZ.run();
    motorY.run();
    motorXL.run();
    motorXR.run();
  }
}

bool getToolPos(int n, float &x, float &y, float &z, float &z1) {
  switch (n) {
    case 1: x = TOOL1_X; y = TOOL1_Y; z = TOOL1_Z; z1 = TOOL1_Z1; return true;
    case 2: x = TOOL2_X; y = TOOL2_Y; z = TOOL2_Z; z1 = TOOL2_Z1; return true;
    default: return false;
  }
}

// Tool 1 uses the wider TOOL1_X_OFFSET (see its #define comment); tool 2
// keeps the plain shared TOOL_X_OFFSET.
float toolApproachOffset(int n) {
  return (n == 1) ? TOOL1_X_OFFSET : TOOL_X_OFFSET;
}

void toolTake(int n) {
  if (!isHomed) { Serial.println(F("Home first: h")); return; }
  if (currentTool != 0) { Serial.println(F("Put current tool first: tool put")); return; }
  float tx, ty, tz, tz1;
  if (!getToolPos(n, tx, ty, tz, tz1)) { Serial.println(F("Unknown tool (1 or 2)")); return; }

  Serial.print(F("Taking tool ")); Serial.println(n);
  blockMoveXYZ(tx + toolApproachOffset(n), ty, tz);
  blockMoveX(tx);
  delay(300); // let the magnet settle
  blockMoveZ(tz1);
  currentTool = n;
  if (n == 2) {
    bladeServo.attach(SERVO_PIN);
  } else if (n == 1) {
    if (bladeServo.attached()) bladeServo.detach();
    analogWrite(LASER_PIN, 0); // start OFF
  }
  Serial.print(F("Tool ")); Serial.print(n); Serial.println(F(" ready"));
}

void toolPut() {
  if (!isHomed) { Serial.println(F("Home first: h")); return; }
  if (currentTool == 0) { Serial.println(F("No tool loaded")); return; }
  float tx, ty, tz, tz1;
  getToolPos(currentTool, tx, ty, tz, tz1);

  int returningTool = currentTool;
  float offset = toolApproachOffset(returningTool);
  Serial.println(F("Returning tool..."));
  blockMoveXYZ(tx + offset, ty, tz1);
  blockMoveX(tx);
  blockMoveZ(tz);
  delay(300); // settle before releasing
  blockMoveX(tx + offset);
  if (returningTool == 2) { bladeOpen(); bladeServo.detach(); }
  currentTool = 0;
  Serial.print(F("Tool ")); Serial.print(returningTool); Serial.println(F(" returned"));
}

void doWeedRow(float rowY) {
  // Reposition to the far end (X_END) FIRST, before Y — this is an
  // X-only move, no cutting yet, at whatever safe travel height Z is
  // currently at. X_END is well clear of the tool rack (which sits near
  // X_START), so it's just as safe a spot to descend as X_START was.
  blockMoveX(WEED_X_END_MM);
  blockMoveY(rowY);
  blockMoveZ(WEED_Z_MM);
  bladeMoveTo(BLADE_CLOSED_DEG);

  motorXL.setMaxSpeed(WEED_RUN_SPEED_X); motorXL.setAcceleration(WEED_RUN_ACCEL_X);
  motorXR.setMaxSpeed(WEED_RUN_SPEED_X); motorXR.setAcceleration(WEED_RUN_ACCEL_X);
  // Sweep back toward X_START (home side) instead of out toward X_END.
  long targetXL = clampAbs(-lroundf(WEED_X_START_MM * STEPS_PER_MM_X), -X_TRAVEL_STEPS, 0);
  long targetXR = clampAbs( lroundf(WEED_X_START_MM * STEPS_PER_MM_X),  0, X_TRAVEL_STEPS);
  long sweepStartSteps = motorXL.currentPosition();
  long cycleSteps     = lroundf(WEED_CYCLE_MM * STEPS_PER_MM_X);
  long halfCycleSteps = cycleSteps / 2;

  motorXL.moveTo(targetXL);
  motorXR.moveTo(targetXR);
  while (motorXL.distanceToGo() != 0 || motorXR.distanceToGo() != 0) {
    motorXL.run();
    motorXR.run();

    long traveled = labs(motorXL.currentPosition() - sweepStartSteps);
    long phase = traveled % cycleSteps;
    int targetOffset;
    if (phase < halfCycleSteps) {
      targetOffset = BLADE_CLOSED_DEG + (int)(((long)(BLADE_OPEN_DEG - BLADE_CLOSED_DEG) * phase) / halfCycleSteps);
    } else {
      long fallPhase = phase - halfCycleSteps;
      targetOffset = BLADE_OPEN_DEG - (int)(((long)(BLADE_OPEN_DEG - BLADE_CLOSED_DEG) * fallPhase) / halfCycleSteps);
    }
    bladeWrite(targetOffset);
  }
  bladeMoveTo(BLADE_OPEN_DEG);
  blockMoveZ(WEED_RETREAT_Z_MM);

  // Sweep already ended at X_START (home side) — no extra retreat move
  // needed, unlike the old low->high version which had to travel back.
}

void doWeedPass() {
  if (!isHomed) { Serial.println(F("Home first: h")); return; }
  if (currentTool != 2) { Serial.println(F("Attach tool 2 first: tool take 2")); return; }

  Serial.println(F("Starting weed pass..."));

  // doWeedRow() repositions to X_END itself before each row's cut, so no
  // separate pre-positioning move is needed here.

  Serial.println(F("Row 1..."));
  doWeedRow(WEED_ROW1_Y_MM);
  Serial.println(F("Row 2..."));
  doWeedRow(WEED_ROW2_Y_MM);

  bladeOpen();
  Serial.println(F("Weed pass complete."));
}

void stopAll() {
  motorZ.stop();  motorZ.setCurrentPosition(0);
  motorY.stop();  motorY.setCurrentPosition(0);
  motorXR.stop(); motorXR.setCurrentPosition(0);
  motorXL.stop(); motorXL.setCurrentPosition(0);
  isHomed = false;
  bladeOpen();
  analogWrite(LASER_PIN, 0);
  Serial.println(F("All motors stopped, positions zeroed."));
}

void setAllSpeed(float spd, float acc) {
  motorZ.setMaxSpeed(spd);  motorZ.setAcceleration(acc);
  motorY.setMaxSpeed(spd);  motorY.setAcceleration(acc);
  motorXR.setMaxSpeed(spd); motorXR.setAcceleration(acc);
  motorXL.setMaxSpeed(spd); motorXL.setAcceleration(acc);
}

void setAllToRunSpeed() {
  motorZ.setMaxSpeed(RUN_SPEED_Z);   motorZ.setAcceleration(RUN_ACCEL_Z);
  motorY.setMaxSpeed(RUN_SPEED_Y);   motorY.setAcceleration(RUN_ACCEL_Y);
  motorXR.setMaxSpeed(RUN_SPEED_X);  motorXR.setAcceleration(RUN_ACCEL_X);
  motorXL.setMaxSpeed(RUN_SPEED_X);  motorXL.setAcceleration(RUN_ACCEL_X);
}

bool switchTriggered(uint8_t pin, uint8_t trigLevel, int samples = 8) {
  for (int i = 0; i < samples; i++) {
    if (digitalRead(pin) != trigLevel) return false;
    delayMicroseconds(300);
  }
  return true;
}

bool homeOneAxis(AccelStepper &motor, uint8_t swPin, uint8_t trigLevel, int dir, long maxSteps, long backoffSteps) {
  if (switchTriggered(swPin, trigLevel, 10)) {
    motor.move((long)(-dir) * HOME_ALREADY_TRIGGERED_BACKOFF);
    while (motor.run()) {
      if (!switchTriggered(swPin, trigLevel, 3)) { motor.stop(); break; }
    }
    delay(100);
  }
  motor.move((long)dir * maxSteps);
  while (motor.distanceToGo() != 0) {
    motor.run();
    if (switchTriggered(swPin, trigLevel, 8)) {
      motor.stop();
      motor.setCurrentPosition(0);
      motor.move((long)(-dir) * backoffSteps);
      while (motor.run()) {}
      motor.setCurrentPosition(0);
      return true;
    }
  }
  return false;
}

bool homeXBoth(int dir, long maxSteps, long backoffSteps) {
  bool xlOn = switchTriggered(SW_XL, SW_TRIG_XL, 10);
  bool xrOn = switchTriggered(SW_XR, SW_TRIG_XR, 10);
  if (xlOn || xrOn) {
    if (xlOn) motorXL.move((long)(-dir) * HOME_ALREADY_TRIGGERED_BACKOFF);
    if (xrOn) motorXR.move((long)(dir) * HOME_ALREADY_TRIGGERED_BACKOFF);
    while (motorXL.distanceToGo() != 0 || motorXR.distanceToGo() != 0) {
      motorXL.run();
      motorXR.run();
      if (xlOn && !switchTriggered(SW_XL, SW_TRIG_XL, 3)) { motorXL.stop(); xlOn=false; }
      if (xrOn && !switchTriggered(SW_XR, SW_TRIG_XR, 3)) { motorXR.stop(); xrOn=false; }
    }
    delay(100);
  }
  motorXL.move((long)dir * maxSteps);
  motorXR.move((long)(-dir) * maxSteps);
  bool xlOK=false, xrOK=false;
  while (motorXL.distanceToGo() != 0 || motorXR.distanceToGo() != 0) {
    if (!xlOK) motorXL.run();
    if (!xrOK) motorXR.run();
    if (!xlOK && switchTriggered(SW_XL, SW_TRIG_XL, 8)) {
      motorXL.stop(); motorXL.setCurrentPosition(0); xlOK=true;
    }
    if (!xrOK && switchTriggered(SW_XR, SW_TRIG_XR, 8)) {
      motorXR.stop(); motorXR.setCurrentPosition(0); xrOK=true;
    }
    if (xlOK && xrOK) break;
  }
  if (!xlOK || !xrOK) return false;
  motorXL.move((long)(-dir) * backoffSteps);
  motorXR.move((long)(dir) * backoffSteps);
  while (motorXL.distanceToGo() != 0 || motorXR.distanceToGo() != 0) {
    motorXL.run();
    motorXR.run();
  }
  motorXL.setCurrentPosition(0);
  motorXR.setCurrentPosition(0);
  return true;
}

void doHoming() {
  isHomed = false;
  setAllSpeed(HOME_SPEED, HOME_ACCEL);

  Serial.println(F("Homing Z..."));
  if (!homeOneAxis(motorZ, SW_Z, SW_TRIG_Z, HOME_DIR_Z, HOME_MAX_STEPS_Z, HOME_BACKOFF_Z)) {
    Serial.println(F("FAIL: Z (ran out of distance, switch never tripped)")); setAllToRunSpeed(); return;
  }
  Serial.println(F("Z OK"));

  Serial.println(F("Homing Y..."));
  if (!homeOneAxis(motorY, SW_Y, SW_TRIG_Y, HOME_DIR_Y, HOME_MAX_STEPS_Y, HOME_BACKOFF_Y)) {
    Serial.println(F("FAIL: Y (ran out of distance, switch never tripped)")); setAllToRunSpeed(); return;
  }
  Serial.println(F("Y OK"));

  Serial.println(F("Homing X..."));
  if (!homeXBoth(HOME_DIR_X, HOME_MAX_STEPS_X, HOME_BACKOFF_X)) {
    Serial.println(F("FAIL: X (ran out of distance, switch never tripped)")); setAllToRunSpeed(); return;
  }
  Serial.println(F("X OK"));

  setAllToRunSpeed();
  isHomed = true;
  Serial.println(F("HOMING COMPLETE (HOMED)"));
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "sw")            { printSwitches();  return; }
  if (cmd == "pos" || cmd == "p") { printPositions(); return; }
  if (cmd == "stop")          { stopAll();        return; }
  if (cmd == "h")              { doHoming();       return; }

  if (cmd.startsWith("xyz ") || cmd.startsWith("m ")) {
    if (!isHomed) { Serial.println(F("Home first: h")); return; }
    int prefixLen = cmd.startsWith("xyz ") ? 4 : 2;
    String rest = cmd.substring(prefixLen); rest.trim();
    int sp1 = rest.indexOf(' ');
    int sp2 = (sp1 >= 0) ? rest.indexOf(' ', sp1 + 1) : -1;
    if (sp1 < 0 || sp2 < 0) { Serial.println(F("Usage: xyz <xmm> <ymm> <zmm>  (or: m <xmm> <ymm> <zmm>)")); return; }
    float xmm = rest.substring(0, sp1).toFloat();
    float ymm = rest.substring(sp1 + 1, sp2).toFloat();
    float zmm = rest.substring(sp2 + 1).toFloat();
    if (xmm < 0) xmm = 0;
    if (ymm < 0) ymm = 0;
    if (zmm < 0) zmm = 0;
    blockMoveXYZ(xmm, ymm, zmm);
    Serial.print(F("Move complete — X=")); Serial.print(xmm); Serial.print(F("mm Y="));
    Serial.print(ymm); Serial.print(F("mm Z=")); Serial.print(zmm); Serial.println(F("mm"));
    return;
  }

  if (cmd.startsWith("tool ")) {
    String rest = cmd.substring(5); rest.trim();
    if (rest.startsWith("take")) {
      toolTake(rest.substring(4).toInt());
    } else if (rest == "put") {
      toolPut();
    } else {
      Serial.println(F("Usage: tool take 1 / tool take 2 / tool put"));
    }
    return;
  }

  if (cmd == "weed") { doWeedPass(); return; }

  if (cmd.startsWith("servo ")) {
    if (!bladeServo.attached()) bladeServo.attach(SERVO_PIN);
    int offsetDeg = constrain(cmd.substring(6).toInt(), BLADE_CLOSED_DEG, BLADE_OPEN_DEG);
    bladeMoveTo(offsetDeg);
    Serial.print(F("Servo -> offset ")); Serial.print(offsetDeg);
    Serial.print(F(" (real angle ")); Serial.print(SERVO_CENTER_DEG + offsetDeg); Serial.println(F(")"));
    return;
  }

  if (cmd.startsWith("laser ")) {
    if (bladeServo.attached()) bladeServo.detach();
    int val = constrain(cmd.substring(6).toInt(), 0, 255);
    analogWrite(LASER_PIN, val);
    Serial.print(F("Laser -> ")); Serial.println(val);
    return;
  }

  int spaceIdx = cmd.indexOf(' ');
  if (spaceIdx < 0) {
    Serial.println(F("Unknown command. Try: z 50 / y 20 / xr 10 / xl 10 / x 100 / xyz 100 50 30 / sw / pos / stop"));
    return;
  }

  String axis = cmd.substring(0, spaceIdx);
  float  mm   = cmd.substring(spaceIdx + 1).toFloat();
  if (mm < 0) mm = 0;

  if (axis == "z" || axis == "y" || axis == "xr" || axis == "xl" || axis == "x") {
    if (!isHomed) { Serial.println(F("Home first: h")); return; }
  }

  if (axis == "z") {
    motorZ.setMaxSpeed(RUN_SPEED_Z); motorZ.setAcceleration(RUN_ACCEL_Z);
    long target = clampAbs(lroundf(mm * STEPS_PER_MM_Z), 0, Z_TRAVEL_STEPS);
    motorZ.moveTo(target);
    Serial.print(F("Z -> ")); Serial.print(target); Serial.print(F(" steps (")); Serial.print(mm); Serial.println(F("mm)"));
  } else if (axis == "y") {
    motorY.setMaxSpeed(RUN_SPEED_Y); motorY.setAcceleration(RUN_ACCEL_Y);
    long target = clampAbs(-lroundf(mm * STEPS_PER_MM_Y), -Y_TRAVEL_STEPS, 0);
    motorY.moveTo(target);
    Serial.print(F("Y -> ")); Serial.print(target); Serial.print(F(" steps (")); Serial.print(mm); Serial.println(F("mm)"));
  } else if (axis == "xr") {
    motorXR.setMaxSpeed(RUN_SPEED_X); motorXR.setAcceleration(RUN_ACCEL_X);
    long target = clampAbs(lroundf(mm * STEPS_PER_MM_X), 0, X_TRAVEL_STEPS);
    motorXR.moveTo(target);
    Serial.print(F("XR -> ")); Serial.print(target); Serial.print(F(" steps (")); Serial.print(mm); Serial.println(F("mm)"));
  } else if (axis == "xl") {
    motorXL.setMaxSpeed(RUN_SPEED_X); motorXL.setAcceleration(RUN_ACCEL_X);
    long target = clampAbs(-lroundf(mm * STEPS_PER_MM_X), -X_TRAVEL_STEPS, 0);
    motorXL.moveTo(target);
    Serial.print(F("XL -> ")); Serial.print(target); Serial.print(F(" steps (")); Serial.print(mm); Serial.println(F("mm)"));
  } else if (axis == "x") {
    motorXL.setMaxSpeed(RUN_SPEED_X); motorXL.setAcceleration(RUN_ACCEL_X);
    motorXR.setMaxSpeed(RUN_SPEED_X); motorXR.setAcceleration(RUN_ACCEL_X);
    long targetXL = clampAbs(-lroundf(mm * STEPS_PER_MM_X), -X_TRAVEL_STEPS, 0);
    long targetXR = clampAbs( lroundf(mm * STEPS_PER_MM_X),  0, X_TRAVEL_STEPS);
    motorXL.moveTo(targetXL);
    motorXR.moveTo(targetXR);
    Serial.print(F("X -> XL=")); Serial.print(targetXL); Serial.print(F(" XR=")); Serial.print(targetXR);
    Serial.print(F(" steps (")); Serial.print(mm); Serial.println(F("mm)"));
  } else {
    Serial.println(F("Unknown axis. Use: z / y / xr / xl / x / xyz"));
  }
}

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(SW_XR, INPUT_PULLUP);
  pinMode(SW_XL, INPUT_PULLUP);
  pinMode(SW_Z,  INPUT_PULLUP);
  pinMode(SW_Y,  INPUT_PULLUP);

  bladeWrite(BLADE_OPEN_DEG);
  pinMode(LASER_PIN, OUTPUT);
  analogWrite(LASER_PIN, 0);

  setAllToRunSpeed();

  Serial.println(F("=== WeedZap Direction/Switch/Homing Test ==="));
  Serial.println(F("Commands: z/y/xr/xl/x <mm>  |  xyz <x> <y> <z>  |  h  |  tool take 1/2  |  tool put  |  weed  |  servo <-5..72>  |  laser <0-255>  |  sw  |  pos  |  stop"));
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
  motorZ.run();
  motorY.run();
  motorXR.run();
  motorXL.run();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}
