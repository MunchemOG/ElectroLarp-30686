# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FTC (*FIRST* Tech Challenge) robot controller project for the DECODE (2025-2026) season, built on top of the official FTC SDK (v11.2.1). It's an Android Studio / Gradle multi-module project:

- `FtcRobotController/` — vendor SDK app module (Qualcomm/FIRST sample code). Do not edit unless intentionally patching the SDK itself; team code never belongs here.
- `TeamCode/` — the team's actual robot code, under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. This is where nearly all work happens.

The drivetrain/path-following layer is [Pedro Pathing](https://pedropathing.com) (the newest pedro3 release along with Ivy both develeoped by the pedro team), plus FrozenMilk's Sloth/Sinister annotation loader and Bylazar's panels dashboard.

## Build & run

This is an Android app deployed to a FTC Robot Controller (REV Control Hub), not something run/tested locally in a JVM.

- Build (from repo root): `./gradlew build`
- Build just TeamCode: `./gradlew :TeamCode:build`
- Install to a connected/ADB-linked Control Hub: `./gradlew :TeamCode:installDebug`
- Clean: `./gradlew clean`
- There is no unit test suite in this repo; verification happens by deploying to the robot and running OpModes from the Driver Station, or via the Pedro Pathing tuning OpModes (see below).

Normal workflow is through Android Studio (Build > Make Project, Run > Run App), which drives the same Gradle tasks.

## Architecture

**OpMode class hierarchy.** All OpModes route through `util/CommandOpMode` (`TeamCode/.../util/CommandOpMode.java`), a thin wrapper around Pedro's ivy `Scheduler`/`Command` system layered on the FTC SDK's `OpMode`. `init()`/`loop()`/`stop()` delegate to `Scheduler.execute()`/`reset()`; concrete OpModes call `schedule(Command...)` to queue behavior rather than hand-rolling state machines.

**Alliance-parameterized autos.** Autonomous logic is written once per starting position (`auto/Close.java`, `auto/Far.java`) and parameterized by `util/Alliance` (`RED`/`BLUE`). Thin subclasses per color/side (`auto/blue/blueClose.java`, `auto/red/redClose.java`, etc.) just call `super(Alliance.BLUE)`/`super(Alliance.RED)` and carry the `@Autonomous` annotation — this is the pattern to follow when adding a new auto variant rather than duplicating logic. Paths for each side live in `auto/paths/` (`ClosePaths.java`, `FarPaths.java`), separate from the OpMode logic itself.

**Teleop** mirrors this: `tele/Tele.java` is alliance-parameterized, `tele/teleBlue.java`/`tele/teleRed.java` are the registered `@TeleOp` entry points.

**Robot/subsystem ownership.** `Robot.java` is the single, central owner of shared hardware/state (currently the Pedro `Follower`); subsystems (`subsystem/Intake.java`, `Shooter.java`, `Turret.java`, `Transfer.java`, `Latch.java`) are meant to be composed onto it rather than each OpMode wiring up hardware independently. Much of this is currently skeleton/stub code (empty classes) being filled in incrementally — check whether a subsystem is actually implemented before assuming its behavior.

**`util/CachedMotor`** wraps `DcMotorEx` and skips redundant hardware calls (`setPower`, `setTargetPosition`, `setZeroPowerBehavior`, etc. all no-op if the value hasn't changed), and caches/filters `getCurrent()` reads through `ScalingLowPassFilter` on a time interval (`currentCacheTimeMS`) since current sensing is comparatively slow. Prefer this over raw `DcMotorEx` for any new motor-driving code. `@Configurable` fields on it (and on other classes like `Intake`, `ArduVision`) are tunable live via the Bylazar/Sloth panels dashboard — treat them as the intended way to expose tunable constants, not hardcoded literals.

**Vision** (`vision/ArduVision.java`) is AprilTag-ID-only (no pose/lens calibration) — it classifies which of two "Cell" states the Hive shows by voting on which tag ID cluster is visible above a decision-margin threshold, defaulting to `TIPPING` when ambiguous. Camera rotation is handled physically, not in software (`SENSOR_NATIVE` must stay the default — see the class's own header comment before changing this).

**Pedro Pathing setup** lives in `pedro/Constants.java` (`Constants.create(HardwareMap)` builds the `Follower` — currently a stub) and `pedro/Tuning.java`. `pedro/procedures/` holds the interactive tuning OpModes (`MecanumTuner`, `OTOSTuner`, `PinpointTuner`, `OctoQuadTuner`, `ThreeWheelTuner`, `ThreeWheelIMUTuner`, `TwoWheelTuner`, `ForesightTuner`, `Tests`) used to calibrate localization/drivetrain on real hardware from the Driver Station — these are run on-robot, not something to unit test.

## Git workflow (team-specific — see `TeamCode/.../teamcode/gitInstructions.md` for full detail)

- `main` mirrors Pedro's upstream template branch (currently `upstream/pedro3`, tracked via a second `upstream` remote pointing at `Pedro-Pathing/Quickstart`) and is not where feature work happens. Feature/season work happens on branches like `Biobuzz` or `Offseason-2026`, which periodically merge `main` down to pick up template/Pedro updates.
- Commit message format: `[day] [type of day] [tag]: short desc`, e.g. `D6 meet feat: added slew rate`.
  - Day: sequential meet/day counter for the season.
  - Type of day: `comp` | `meet` | `scrim`.
  - Tag: `test` | `feat` | `fix` | `chore` | `docs`.
