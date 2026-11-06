# Autonomous with the Pedro visualizer

## Test a new path (3 steps)

1. **Export** from the Pedro visualizer and save it as a new file in `opmodes/auto/paths/`,
   e.g. `RedClosePaths.java` (the outer class name must match the file name).
2. **Fix 2 things** in that file:
   - the `package` line -> `package org.firstinspires.ftc.teamcode.opmodes.auto.paths;`
   - Find & Replace in the file (Ctrl+R): `com.pedropathing.` -> `org.firstinspires.ftc.teamcode.pedro.visualizer.`
     (only the imports contain it). Delete any import Android Studio marks red that the paths don't use.
3. In `BasicAuto.java` change the one line: `PATHS = RedClosePaths.class;`

Place the robot at the pose shown under **PLACE ROBOT AT** in init (the start of the first path).

Why step 2: the visualizer writes Pedro **2.x** code (`pathBuilder()`, `BezierLine`, `PathChain`),
this project runs Pedro **3.0**, which has none of those. The `pedro/visualizer` package has
look-alike classes that turn the export into Pedro 3.0 paths. The export's code itself is untouched.

## What the robot does on each path

Set in `BasicAuto`:

| setting | effect |
|---|---|
| `PLAN = ""` | path 1, 3, 5 ... score (drive + spin up, then shoot) / path 2, 4 ... collect (intake on). A final collect path becomes park. |
| `PLAN = "SCSCP"` | one letter per path: **S** score, **C** collect, **D** drive only, **P** park (all off) |
| `SHOOT_PRELOAD_FIRST = true` | shoot before the first path (robot starts in shooting position); the default pattern then starts with collect |

## Custom routine

Make another class `extends BaseAuto` and write `routine(...)` yourself with the steps in
`AutoActions` (`scoreAt`, `collect`, `follow`, `shoot`, `intakeOn`, `intakeOff`, `spinUp`,
`waitSeconds`, `park`, `stopAll`) plus Ivy `sequential` / `parallel`. RedClose by index `p.get(0)`
or by field name `p.get("Path3")`.

## Tuning (FTC Dashboard, `final_Globals`)

`AUTO_TURRET_POS`, `AUTO_SPINUP_TIMEOUT_S`, `AUTO_SHOT_TIMEOUT_S`, `AUTO_PATH_TIMEOUT_S`,
`AUTO_INTAKE_SETTLE_S`. Shot velocities / feed timing are the TeleOp values (`TELE_*_VELOCITY`, `SHOOT_*`).
