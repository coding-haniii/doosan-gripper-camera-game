
## Test 1: whole game, no camera / real robot

```bash
bash scripts/run_paper_tictactoe.sh --source demo
```

Robot X starts. Human chooses O by 1..9. X is minimax, games end by result; X win triggers mock celebration, any finish triggers mock home. Images: `out/paper_tic_turn_*.png`, `out/paper_tic_final.png`.

```bash
bash scripts/run_paper_tictactoe.sh --source demo --moves 1 2 3 4 5 6 7 8 9
xdg-open out/paper_tic_final.png
```

## Test 2: webcam + physical paper, MOCK arm

```bash
bash scripts/run_paper_tictactoe.sh --source camera --camera-index 0
```

1. Clamp top-down camera and clean empty 3x3 paper grid; avoid reflections/shadows; board should occupy much of image.
2. Click four *outer* corners of the board, in order **top-left, top-right, bottom-right, bottom-left**. (This is image-to-paper homography ONLY, not robot calibration.)
3. Press SPACE to capture blank board baseline.
4. Robot prints X location and mocks 2 pen strokes. **Operator draws this X manually on paper** (because mock has no actuator); press SPACE to capture updated baseline. Missing X causes a safe stop.
5. Human draws one O in an empty cell, removes pen/hand from frame, presses SPACE. The vision module compares with the baseline, checks a ring shape, and identifies one new O. Confirm detected cell with `y` in terminal. Reject and recapture otherwise.
6. Repeat until X win, O win, or draw. Press `q` in image window or Ctrl+C to quit.

Use thick dark marker and roughly centered O; the classifier is deliberately conservative. It may reject uneven, faint, rotated or tiny circles. It will never automatically accept two new cells.

## Limitations / safety

This is **operator-assisted camera input**, not autonomous continuous tracking. It relies on stable, flat, preprinted 3x3 grid; camera must remain fixed, illumination stable. Image-to-robot registration (hand-eye calibration / fiducial markers), robot tool/pen height, pen-up separation, board movement detection, motion planning, gripper actuation and emergency-stop/supervisory control are NOT implemented/validated.

`--robot-mode real` deliberately raises `NotImplementedError` even with `--execute-motion`: DO NOT turn on actual arm based only on webcam pixel coordinates. For real operation, independently commission camera-to-base calibration, robot pose, clearance, stroke trajectories and motion safety; a qualified integrator must explicitly implement/enable a verified hardware adapter.

Card 1 still triggers the old one-turn demo in `robot_games/app.py`. To integrate card 1 with this camera loop, modify `app.py` dispatch with a persistent supervisor after hardware commissioning. Don't accidentally redirect the card to a real arm in an unverified cell.
