# OpenPnP Rotary Axes (A / B) — `OPENPNP_ROTARY_AXES`

This fork adapts Marlin bugfix-2.1.x to drive an OpenPnP pick-and-place head
on a RAMPS 1.5 board. Two nozzle-rotation steppers are wired to the free
`E0`/`E1` sockets (RAMPS has no native A/B/C axis sockets), and this feature
exposes them to G-code as clean `A`/`B` rotary axes instead of `E`/`T`.

## Hardware

| Function              | Wiring                                   |
|-----------------------|-------------------------------------------|
| X / Y / Z             | Standard linear axes                      |
| Nozzle 1 rotation     | `E0` socket (A4988), converted-bipolar 28BYJ-48 |
| Nozzle 2 rotation     | `E1` socket (A4988), converted-bipolar 28BYJ-48 |
| Vacuum pump           | D8 (`M42 P8`)                             |
| Vacuum valve 1 / 2    | D9 / D10 (`M42 P9` / `M42 P10`)           |
| Top camera light      | AUX MOSFET output                         |
| Bottom camera light   | Servo / AUX MOSFET output                 |
| Controller            | RAMPS 1.5, COM3 @ 250000 baud              |

`Configuration.h` sets `EXTRUDERS 2` — these are not real extruders
(`TEMP_SENSOR_0`/`TEMP_SENSOR_1` are `0`, `PREVENT_COLD_EXTRUSION` is
disabled); the two "extruder" slots are purely a vehicle for driving the two
E-socket steppers through Marlin's existing multi-extruder machinery.

## What it does

With `#define OPENPNP_ROTARY_AXES` enabled (`Configuration.h`):

```
G1 A90 F300   ; selects Tool 0 internally, moves E0 to 90
G1 B180 F300  ; selects Tool 1 internally, moves E1 to 180
G92 A0        ; zeroes nozzle 1's position (selects Tool 0 first)
G92 B0        ; zeroes nozzle 2's position (selects Tool 1 first)
M114          ; reports "X:.. Y:.. Z:.. A:.. B:.." instead of "...E:.."
```

- `A` always means "Tool 0's E axis", `B` always means "Tool 1's E axis" —
  the tool switch happens automatically and only when the tool isn't
  already active (so repeated `A`/`B` moves don't re-trigger a planner
  sync every line).
- Plain `E`/`T` G-code is completely unaffected — this is a pure addition,
  not a replacement, of the existing E-axis path.
- If a line contains both `A` and `B` (e.g. `G1 A10 B20`), `A` wins and `B`
  is silently ignored — Marlin can only move one extruder stepper per
  planner block, so simultaneous independent A+B motion in one line isn't
  possible with this approach (see **Limitations** below).
- When `OPENPNP_ROTARY_AXES` is undefined, none of this code is compiled in
  — behavior is byte-for-byte stock Marlin.

## Why M114 needed extra work

Marlin tracks **one shared E position** internally — there's no per-extruder
position slot anywhere in the stack (parser → `motion.position.e` →
planner block → stepper ISR). Switching tools only changes which physical
stepper receives step pulses, not "which E value is current." So naively
relabeling `E:` as `A:`/`B:` would only ever show whichever nozzle moved
*last*.

To make `M114` (and OpenPnP's position-confirmation polling) see both
nozzles correctly at all times, `GcodeSuite::rotary_axis_position[2]`
tracks each tool's own last-commanded target independently, updated
whenever `G1 A/B` or `G92 A/B` runs. `M114` and `G92`'s own auto-echo both
read from this array via `GcodeSuite::report_openpnp_position()`.

## Files touched (all gated by `#if ENABLED(OPENPNP_ROTARY_AXES)`)

- `Marlin/Configuration.h` — the feature flag, next to `EXTRUDERS 2`.
- `Marlin/src/gcode/gcode.h` / `gcode.cpp` — `rotary_axis_position[2]`,
  `report_openpnp_position()`, and the `A`/`B` parsing branch in
  `get_destination_from_command()` (used by `G0`/`G1`).
- `Marlin/src/gcode/geometry/G92.cpp` — same `A`/`B` aliasing for
  position-zeroing (uses `tool_change(tool, true)` — `no_move=true`,
  since `G92` must never physically move anything).
- `Marlin/src/gcode/host/M114.cpp` — defines `report_openpnp_position()`,
  used instead of `motion.report_position_projected()`.

Deliberately **not** touched: `planner.cpp`, `stepper.cpp`, or `motion.cpp`'s
shared `report_logical_position()` helper (used by several other reporting
call sites) — the M114/G92 reporting change is fully self-contained in the
files above.

## Calibrating steps-per-degree

No firmware change needed — Marlin already stores independent
steps-per-unit per extruder and `M92` already accepts a `T` parameter to
target a specific extruder without switching the active tool:

```
M92 T0 E<steps_per_degree_nozzle1>
M92 T1 E<steps_per_degree_nozzle2>
M500
```

## Limitations

- **No simultaneous A+B motion in one line.** Each is really "select tool,
  move the shared E axis" under the hood — `A` and `B` can't move
  independently and concurrently within a single G-code move.
- **Shared E slot in relative mode across tool switches.** If you drive `A`
  and `B` in relative E mode (`M83`) and interleave tool switches, both
  share the same underlying position accumulator — there's no per-tool
  relative-mode memory. Only ever exercised/tested in absolute mode
  (`M82`) so far.
- **`rotary_axis_position` reflects last *commanded* target, not live
  stepper position.** Accurate in normal command sequencing (each move
  synchronizes before the next command), but not a substitute for a true
  per-axis step counter.

## OpenPnP-side configuration (for reference)

- `GcodeDriver`: port `COM3`, baud `250000`.
- Define two axes of type **Rotation** (units = degrees), letters `A`
  and `B`.
- Position-confirmation regex per axis: `A:([^ ]*)` / `B:([^ ]*)` against
  the `M114` response.

## Verified on hardware

- `G1 A/B` tool switching + absolute and relative moves.
- Round-trip tool switching (`A` → `B` → `A`) with correct absolute targets.
- `G92 A0` / `G92 B0` independently zeroing each nozzle.
- `M114` and `G92`'s auto-echo consistently showing `X Y Z A B`.
- Plain `G1 E...` unaffected by the alias path.
- Build passes with the flag both enabled and disabled.

## Not yet implemented

- Vacuum pump / valve control (`M42` on D8/D9/D10) — no G-code changes made,
  stock `M42` already covers this, just needs OpenPnP actuator config.
- Camera light outputs (AUX MOSFET / servo) — same, likely no firmware
  change needed, just OpenPnP actuator/light config.
