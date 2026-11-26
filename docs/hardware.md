# From a calibrated follower to the first stroke

## Install and identify the arm

Use a Python 3.12 environment. Start from a supported resting pose. Check the motor IDs and calibration using the official follower setup. Keep using the same robot ID on this machine. The simulation and template commands do not open a serial port. [SO-101 setup and calibration](https://huggingface.co/docs/lerobot/en/so101)

```sh
python -m pip install -e '.[hardware]'
lerobot-find-port
# Set these for your own arm. The port below is an example on Linux.
export PORT=/dev/ttyACM0
export ARM=portrait_follower
lerobot-calibrate --robot.type=so101_follower --robot.port="$PORT" --robot.id="$ARM"
```

## Teach the writing surface

1. Print the [US Letter registration guide](../assets/registration-letter.pdf) in landscape at **100% / actual size**. For A4, run `python -m portrait template` and print `out/registration.svg` on landscape A4 at the same scale. Check that the outside crosses are 136 mm apart horizontally and 108 mm vertically. Place the guide at the height the final paper will occupy. Use tape or registration stops so you can exchange it for clean paper without shifting the board.
2. Clamp the base. Put the entire rectangle in comfortable reach, with the top row on the far side. Keep tape, clips and other objects outside the travel area. Tape the uncapped marker to the fixed jaw so it stays mounted when torque is off. Use a small amount of nib compliance. Keep fingers outside the sweep during powered motion.
3. With the arm supported and its joints unpowered, hand-guide the nib to each of the nine crosses. The program asks for two readings per cross. Take the first with the nib just touching at writing pressure. Put an 8 mm spacer on the same spot for the second. Keep the wrist roll and marker inclination similar at every reading. A faint dot on the template confirms contact.
4. If the software rejects the grid, check the row/column order, spacer, wrist posture and print scale. Completion writes `local/<id>/black.json`. Retaping the marker or moving the board invalidates that measurement. To teach again, move the old colour profile to a backup filename.

```sh
python -m portrait teach --port "$PORT" --id "$ARM" --color black
```

## Rehearse and check ink

Start each batch above the top-left cross (R1C1) in its taught hover pose. With torque off, support the arm there and remove the spacer before enabling motion. The controller seeds motor goals from the current pose and holds at the end. Once it is holding, the operator can step away. It leaves the gripper motor alone.

```sh
# Offline route check
python -m portrait run --id "$ARM" --color black --mode air --count 1
# The same route on the arm with the tip raised
python -m portrait run --port "$PORT" --id "$ARM" --color black --mode air --count 1 --execute
# With sacrificial paper in position, make a centred horizontal dash
python -m portrait run --port "$PORT" --id "$ARM" --color black --mode swatch --execute
```

The dash should be about 11 mm long and centred on R2C2. Confirm a continuous line and a clean lift. Put the final sheet against the same registration stops. Start by checking one stroke, then draw small batches. These examples run the first six black strokes.

```sh
python -m portrait run --port "$PORT" --id "$ARM" --color black --mode draw --start 0 --count 1 --execute
python -m portrait run --port "$PORT" --id "$ARM" --color black --mode draw --start 1 --count 5 --execute
```

Continue black from index 6 through 152. For green, grey and orange, support the arm and release the five arm joints, change the marker, and teach that colour with the guide in the same position. Repeat the air and scrap checks. Each colour starts at index zero. Green ends at 44, grey at 27 and orange at 44. Use `--color gray` for the grey marker. Draw the orange layer again only if inspection warrants it.

```sh
python -m portrait release --port "$PORT" --id "$ARM" --supported
```

## If a stroke stops

Ctrl-C requests a hold at the measured pose. A serial failure can also prevent a hold command from arriving. Keep the physical power switch accessible. Support the arm before releasing it, particularly if the tip stopped on paper. Each executed batch leaves a JSONL log under `local/<id>/logs/` with the labels, point indices, commanded and observed angles, and a completion record on success. The local stroke index in a log is relative to that batch’s start. Inspect the page before choosing the next `--start`. Clearing a partial mark may require fresh paper.

The planner checks the taught grid, joint limits and motor calibration. The physical operator verifies clearance around the arm links and mount, which the portable planner does not model. A whole-page rehearsal can be run by giving the full colour count in air mode.
