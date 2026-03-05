#pragma once
#include <stddef.h>

// =============================================================================
// IK CORNER POINTS — for use with code 888
// =============================================================================
// Each row defines one corner (or pen-state transition) of a letter.
// Record these by:
//   1. Send code 200 (limp mode) and physically move the arm to each corner.
//   2. Press ENTER to read the logical degrees for IDs 1, 2, 4, 5, 6.
//   3. Enter those values here in sequence.
//
// pen field:
//   0 = PEN UP   — arm travels to this point without touching the board.
//                  Use this for the first corner of each stroke and for
//                  moves between letters / strokes.
//   1 = PEN DOWN — arm draws TO this point (pen is lowered after arriving
//                  at the previous pen=0 point, OR stays down from prior
//                  pen=1 point).
//
// Typical stroke sequence:
//   {j1, j2, j4, j5, j6, 0}   ← travel to stroke start (pen up)
//   {j1, j2, j4, j5, j6, 1}   ← first drawn corner (pen lowers here)
//   {j1, j2, j4, j5, j6, 1}   ← second drawn corner
//   ...
//   {j1, j2, j4, j5, j6, 0}   ← travel to next stroke start (pen lifts here)
//
// ID 3 is always mirrored from ID 2 automatically — do NOT specify it here.
// =============================================================================

struct IKPoint {
  float   j1;   // Servo ID 1 — logical degrees
  float   j2;   // Servo ID 2 — logical degrees (ID 3 mirrored automatically)
  float   j4;   // Servo ID 4 — logical degrees
  float   j5;   // Servo ID 5 — logical degrees
  float   j6;   // Servo ID 6 — logical degrees
  uint8_t pen;  // 0 = pen UP (travel), 1 = pen DOWN (draw)
};

// -----------------------------------------------------------------------------
// EDIT BELOW: replace the placeholder points with your recorded corners.
// -----------------------------------------------------------------------------
const IKPoint kIKPoints[] = {
  // ── Example: blocky letter "L" (two strokes) ─────────────────────────────
  // Stroke 1: vertical bar — top to bottom
  // {180.0f, 213.0f,  91.0f, 180.0f, 212.0f, 0},  // travel to top of L
  // {180.0f, 213.0f,  91.0f, 180.0f, 212.0f, 1},  // pen down at top
  // {180.0f, 210.0f, 100.0f, 180.0f, 210.0f, 1},  // pen down at bottom-left corner
  //
  // Stroke 2: horizontal bar — bottom-left to bottom-right
  // {180.0f, 210.0f, 100.0f, 180.0f, 210.0f, 0},  // travel (pen up) — already here, just lifts pen
  // {180.0f, 208.0f, 100.0f, 180.0f, 208.0f, 1},  // pen down — draws right foot of L
  //
  // ── PLACEHOLDER — replace with real values ───────────────────────────────
  {180.0f, 213.0f, 91.0f, 180.0f, 212.0f, 0},  // Point 0: travel to start
};

const size_t kIKPointCount = sizeof(kIKPoints) / sizeof(kIKPoints[0]);
