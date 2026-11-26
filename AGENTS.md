# Working with the portrait

The human supplies the desk, fits the markers, and can stop the arm. Claude Code runs the commands from this checkout, inspects the results, and keeps track of which strokes landed. Use the active virtual environment or invoke its Python directly.

1. Start with `python -m portrait sim`. Open `out/sim/portrait.png`, read `out/sim/report.json`, and check the green heart, grey hatching and both lines of the orange signature. Expect 153 black, 45 green, 28 grey and 45 orange paths. Simulation requires no USB port.
2. For a physical request, open `docs/hardware.md`. Use the operator’s port and robot ID. Motor calibration belongs to that arm. Page profiles belong to the particular marker, grip and fixed board. Ask the operator for physical measurements and the image of a scratch mark when these are needed.
3. After each marker change, teach the profile, rehearse in air, then draw the registration dash on scrap at the height of the final page. Inspect its position and continuity. Remove the scrap while the arm is held clear. Agree on the intended colour and batch before physical motion.
4. Preview the batch by omitting `--execute`. Start with one real stroke. Increase `--count` after checking where it landed. Give `--start` as a zero-based index within that colour. Read the motor log and compare a photograph of the page before deciding whether to repeat or advance.
5. After interruption, inspect the incomplete stroke and the last measured position. A completed motor command does not establish that a marker left ink. Support the arm before releasing torque. Never replay another rig’s raw servo readings.
