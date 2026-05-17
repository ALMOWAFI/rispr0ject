# Lab Tuning Checklist — Tomorrow's Session

Print this or keep it open on the lab machine.
Each section is a parameter to verify and/or tune, in priority order.

---

## 0. Pull and Build

```bash
cd ~/catkin_ws/src/memory_game && git pull origin main
cd ~/catkin_ws && catkin_make --pkg memory_game
source ~/catkin_ws/devel/setup.bash
```

Then start `roscore` in one terminal and the camera + vision in another:

```bash
rosparam load ~/catkin_ws/src/memory_game/config/game_params.yaml
rosrun memory_game vision_node
```

Open `rqt_image_view` and subscribe to `/vision/debug_overlay`.

---

## 1. Verify Image Crop (most important)

**What to verify:** the green rectangle on the overlay should tightly wrap
the play area (just the table area where the blocks sit).

**Currently in yaml:**
```yaml
image_crop_x_min: 280
image_crop_x_max: 1000
image_crop_y_min: 400
image_crop_y_max: 660
```

(These were eyeball estimates for 1280x720 — likely needs adjustment.)

**How to tune:**
1. Hover your mouse over the top-left corner of the play area in
   `rqt_image_view`. Bottom of the window shows `x=NNN, y=NNN`.
2. Note those numbers → those are your new `x_min` and `y_min`.
3. Hover over the bottom-right corner. Note → `x_max` and `y_max`.
4. Edit `~/catkin_ws/src/memory_game/config/game_params.yaml`.
5. Reload: `rosparam load ~/catkin_ws/src/memory_game/config/game_params.yaml`
6. Restart vision (Ctrl+C, then `rosrun memory_game vision_node`).
7. Check overlay — green rectangle should now match your hovered corners.

**First sanity check before tuning anything else:** what is the
camera's actual resolution? If it isn't 1280x720, the values above
are wrong:
```bash
rostopic echo -n1 /realsense/color/image_raw/width
rostopic echo -n1 /realsense/color/image_raw/height
```

---

## 2. Verify Each Color Detects Properly

**What to verify:** each block gets a white circle drawn at its center
in the overlay, with the correct color label.

**If a block is NOT detected:**
- Check if it's INSIDE the green crop rectangle (if not, fix crop first).
- Check `/vision/debug_mask` — if the block is missing from the mask,
  the HSV range is too tight for that block under current lighting.
- Solution: measure actual HSV values for that color (see step 3).

**If a non-block object IS detected:**
- If it's inside the crop area, the HSV range is too loose.
- Solution: measure actual HSV values for that color (see step 3).

---

## 3. Measure Real HSV Values for Each Color

**Tool:** `~/hsv_sampler.py` (already on your machine).

**Step-by-step:**

1. Save a frame from the live camera:
```bash
rosrun image_view image_saver image:=/realsense/color/image_raw _filename_format:=/tmp/frame_%04i.jpg
# Press Ctrl+C after one frame is saved
```

2. Run the sampler:
```bash
python3 ~/hsv_sampler.py /tmp/frame_0000.jpg
```

3. Click on each block 2-3 times (center, corner, near edge).
   - Output: `H xx-yy, S xx-yy, V xx-yy` and a yaml line.

4. Copy the combined yaml line into the appropriate color in
   `config/game_params.yaml` under `hsv_ranges:`:
```yaml
hsv_ranges:
  green:  [<your line>]
  blue:   [<your line>]
  yellow: [<your line>]
```

5. Reload params, restart vision, verify detection works.

**Reference (from phone photo, may differ from real RealSense):**
- Blue measured: `[102, 108, 126, 184, 79, 166]`
  - Padded suggestion if real values close: `[97, 115, 100, 220, 60, 200]`

---

## 4. Tune max_depth_m

**Current value:** `1.00`

**Verify:**
- Run with the current value.
- Look at the depth values published in `/detected_blocks`:
```bash
rostopic echo -n1 /detected_blocks | grep "z:"
```

- If real blocks read at depth significantly less than 1.0m (say,
  most are 0.5-0.7m), you can tighten further to `0.80` for extra
  defense against background.
- If any real block is rejected for being too far (no circle on
  overlay), loosen to `1.20` or `1.50`.

---

## 5. Tune max_block_area (if needed)

**Current value:** `60000` (loose, eyeball-set from March)

**Verify (only if false positives are getting through the crop):**
- In `rqt_image_view`, eyeball the pixel size of a real block.
  Maybe it's ~50x50 = 2500 pixels area.
- A reasonable upper bound: ~3x that = `15000`.

**Suggested change if needed:**
```yaml
max_block_area: 15000
```

Don't change unless you observe a false positive that's larger than
a real block (e.g., a colored shirt slipping through).

---

## 6. Tune max_candidate_jump_px (if needed)

**Current value:** `120`

**Verify (only if vision keeps "losing" a block on small disturbances):**
- A jump of 120 pixels (~10% of image width) is loose.
- If you bump a block by 1-2 cm and vision loses tracking, the jump
  is being interpreted as "different block" too easily.
- Tighter value: `60` (assumes blocks rarely move much).

---

## 7. Verify Smoother (jump-rejection)

**Currently in yaml:**
```yaml
max_position_jump_m: 0.05
position_reset_timeout_sec: 0.5
```

**Scenario to test:** cover a block with your hand briefly, then uncover.

**Expected behavior:**
- White circle disappears (no fresh detection during occlusion).
- Within 0.5s of uncovering, white circle reappears at the block.

**If circle reappears in a wrong location:**
- Check if a similar-colored object is in view during occlusion.
- If yes, that's a false positive — extra_exclusions might be needed.
- If no, the smoother may be too lax — reduce `max_position_jump_m: 0.03`.

**If real blocks are sometimes "blinking" (circle disappears unexpectedly):**
- Smoother might be too strict — increase `max_position_jump_m: 0.08`.
- Or check if blocks are being bumped/moved during play.

---

## 8. Verify Base Exclusion (LED filter)

**Current value:** `base_exclusion_radius_m: 0.20`

**Verify:**
- Look at overlay near the robot base.
- The blue Panda LED should NOT be detected as a blue block.

**If LED IS detected:**
- Bump to `0.25` or `0.30`.
- May need to also check that the actual LED position relative to
  panda_link0 is what we think it is (not at origin if the LED is on
  the back of the base).

---

## 9. Verify Extra Exclusions

**Currently in yaml:**
```yaml
extra_exclusions:
  - { x: 0.65, y: -0.20, z: 0.0, radius: 0.25 }   # behind blue corner
  - { x: 0.65, y: 0.40,  z: 0.0, radius: 0.25 }   # behind green edge
```

**Verify:**
- With image crop active, these MIGHT not be needed anymore (crop
  filters their pixel area).
- If you see false positives at those locations, the zones are working.
- If you NEVER see anything there, the zones are inactive but harmless.

**If a NEW false positive appears at a different location:**
- Echo `/detected_blocks` while the false positive is showing.
- Note the x, y, z of the bad detection.
- Add a new entry to `extra_exclusions` with that center and radius 0.20.

---

## 10. End-to-End Game Test

After vision is tuned, run the full system:

```bash
# Terminal 1: roscore (still running from earlier)
# Terminal 2: camera (still running)
# Terminal 3: vision (still running)

# Terminal 4: player selection
rosparam load ~/catkin_ws/src/memory_game/config/game_params.yaml
rosrun memory_game player_selection.py

# Terminal 5: motion
rosrun memory_game motion_moveit_node _planning_group:=arm

# Terminal 6: game
rosrun memory_game game_node
```

Watch:
- Vision detects all 3 blocks correctly.
- Game generates a sequence and dispatches.
- Robot performs hover-point-hover on each block in order.
- After sequence, player removes blocks → game detects selections.
- Score updates correctly.
- Game ends or starts a new round.

---

## Quick Reference — All Tunable Vision Parameters

| Param | Current | Lower bound | Upper bound | When to change |
|---|---|---|---|---|
| `image_crop_x_min` | 280 | 0 | image width | Crop doesn't wrap play area |
| `image_crop_x_max` | 1000 | 0 | image width | Same |
| `image_crop_y_min` | 400 | 0 | image height | Same |
| `image_crop_y_max` | 660 | 0 | image height | Same |
| `max_depth_m` | 1.00 | 0.5 | 3.0 | Far blocks getting rejected, or backgrounds still slipping |
| `max_block_area` | 60000 | 5000 | 80000 | Big objects slip through crop |
| `min_block_area` | 500 | 100 | 2000 | Real blocks rejected when partially covered (lower) |
| `max_candidate_jump_px` | 120 | 30 | 300 | Tracking lost on small movements (raise), or false positives jumping (lower) |
| `max_position_jump_m` | 0.05 | 0.02 | 0.20 | Smoother too strict (raise) or too lax (lower) |
| `position_reset_timeout_sec` | 0.5 | 0.2 | 2.0 | Slow re-lock (lower) or quick acceptance of false positives (raise) |
| `base_exclusion_radius_m` | 0.20 | 0.10 | 0.35 | LED still detected as blue |
| `smoothing_alpha` | 0.35 | 0.1 | 0.8 | Tracking laggy (raise) or jittery (lower) |

## HSV Range Reference

| Color | Current range | Notes |
|---|---|---|
| green | `[40, 85, 70, 255, 50, 255]` | Wide H — could tighten |
| blue | `[95, 135, 70, 255, 50, 255]` | Wide H — could tighten |
| yellow | `[18, 40, 100, 255, 80, 255]` | Wide H |
| red | `[0,15,90,255,50,255]` + `[165,180,90,255,50,255]` | Disabled by default |

Tighten based on `~/hsv_sampler.py` measurements with the real camera.

---

## Workflow Reminder

For ANY parameter change:
```bash
# 1. Edit ~/catkin_ws/src/memory_game/config/game_params.yaml
# 2. Reload to param server:
rosparam load ~/catkin_ws/src/memory_game/config/game_params.yaml
# 3. Restart vision:
# (Ctrl+C in vision terminal)
rosrun memory_game vision_node
```

The motion node and game node don't need restart for vision-only changes.
