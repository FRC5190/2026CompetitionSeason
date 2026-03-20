# 2026 Competition Season

Java/WPILib command-based robot code for Team 5190's 2026 robot. The active stack is a YAGSL-based swerve drivetrain with Limelight vision, a turreted shooter, REV motor controllers, and PathPlanner autonomous routines.

## Current State

The repository already has the main robot skeleton in place:

- field-oriented swerve teleop driving
- PathPlanner auto loading
- Limelight-backed vision data
- intake, indexer, and turret subsystems
- a `ShootOnTheMove` prototype that blends odometry and vision

It is still an in-progress branch, not a cleaned competition release. The biggest issues to know up front are:

- `src/main/java/frc/robot/commands/ShootAuto.java` is incomplete and may prevent a clean build
- WPILib template files are still present (`Autos`, `ExampleCommand`, `ExampleSubsystem`)
- `RobotContainer` uses controller port `1` while `Constants.java` says `0`
- `Constants.maximumSpeed` is set to `Units.feetToMeters(1)`, which is likely a placeholder
- `Turret` rotation uses CAN ID `17`, and `src/main/deploy/swerve/swervedrive.json` also assigns `17` to the Pigeon2 IMU

## Build, Sim, and Deploy

This project uses GradleRIO `2026.2.1` and targets Java 17. The wrapper in this checkout is not executable, so the safest invocation is:

```bash
bash gradlew build
bash gradlew test
bash gradlew simulateJava
bash gradlew deploy
```

Deployment uses team `5190` from `.wpilib/wpilib_preferences.json`. Desktop and roboRIO debug launch profiles live in `.vscode/launch.json`.

## Runtime Architecture

### Entry and scheduler flow

- `src/main/java/frc/robot/Main.java` is the standard WPILib entry point and only calls `RobotBase.startRobot(Robot::new)`.
- `src/main/java/frc/robot/Robot.java` is a thin `TimedRobot` shell. It creates one `RobotContainer`, runs `CommandScheduler` in `robotPeriodic()`, schedules the selected autonomous command in `autonomousInit()`, cancels it in `teleopInit()`, and cancels everything in `testInit()`.
- Most former Limelight and SmartDashboard debug code in `Robot.java` is commented out rather than removed.

### `RobotContainer` wiring

`src/main/java/frc/robot/RobotContainer.java` is the composition root. It instantiates:

- `SwerveSubsystem`
- `VisionSubsystem`
- `Intake`
- `Indexer`
- `Turret`
- `Superstructure`

It also defines the default drive command, controller bindings, alliance-specific shooting targets, and the autonomous sequence.

### Default drive behavior

The default drive command is built from `SwerveInputStream`:

- left stick Y/X drive translation
- right stick X controls rotation
- deadband comes from `OperatorConstants.DEADBAND` (`0.05`)
- translation is scaled to `0.8`
- alliance-relative control is enabled

`driveDirectAngle` also exists, but it is currently unused.

## Teleop Controls

Current active bindings in `RobotContainer` are:

- `A`: intake roller forward via `superstructure.jogRoller(0.6)`
- `B`: intake roller reverse via `superstructure.jogRoller(-0.6)`
- `Right Bumper`: intake extension out via `superstructure.jogExtension(0.2)`
- `Left Bumper`: intake extension in via `superstructure.jogExtension(-0.2)`
- `Left Trigger`: run `new ShootOnTheMove(turret, drivebase, vision, getShootTarget())`

Commented-out bindings show intended future operator flows for:

- indexer jogging
- AprilTag-assisted drive alignment to specific field tags

## Commands and Coordination

### `src/main/java/frc/robot/Superstructure.java`

`Superstructure` is a thin command factory, not a full finite-state machine. It wraps subsystem actions into schedulable commands using `StartEndCommand` and `RunCommand(...).until(...)`.

Available helpers:

- `jogRoller`
- `jogExtension`
- `jogIndexer`
- `runFlywheel`
- `jogHoodUp`
- `jogHoodDown`
- `setHoodPosition`
- `jogRotationLeft`
- `jogRotationRight`
- `setRotationPosition`
- `autoAim`

`autoAim()` only rotates the turret from vision input. It does not command hood, flywheel, or indexer, so it is only a partial aiming utility.

### `src/main/java/frc/robot/commands/AlignTurretToTag.java`

This command is a simple AprilTag alignment loop for the turret:

- if no tag is visible, the turret rotates slowly to search
- if a tag is visible, `vision.getTX()` is multiplied by `kP = 0.02`
- the command ends when horizontal error is within `1.0` degree

### `src/main/java/frc/robot/commands/ShootOnTheMove.java`

This is the most advanced behavior in the repo. Each cycle it:

1. reads field-relative chassis velocity from the swerve drive
2. latency-compensates the robot pose
3. estimates shot distance from Limelight target-space data or odometry fallback
4. interpolates flywheel output from a three-point distance table
5. computes a compensated shot vector that accounts for robot motion
6. derives turret heading and hood pitch from that shot vector
7. commands turret rotation, hood position, and flywheel output

It runs until interrupted. The structure is strong, but the constants and interpolation table are clearly marked as tune-on-robot values.

### `src/main/java/frc/robot/commands/ShootAuto.java`

This file is intended to implement an autonomous shot sequence:

- align turret to an AprilTag
- spin up the flywheel
- wait for speed
- run the indexer
- stop motors

Right now it is not complete:

- the method signature omits `VisionSubsystem`
- the body still references `vision`
- `withTimeout` is misspelled as `withTimout`
- closing syntax is missing

`RobotContainer` still calls it from autonomous, so this is the main code path to repair before relying on auto shooting.

### Template command files

These files are still stock WPILib scaffolding:

- `src/main/java/frc/robot/commands/Autos.java`
- `src/main/java/frc/robot/commands/ExampleCommand.java`

## Subsystems

### `src/main/java/frc/robot/subsystems/SwerveSubsystem.java`

This is the drivetrain and localization core. It:

- loads YAGSL config from `src/main/deploy/swerve`
- builds the `SwerveDrive` with `SwerveParser`
- loads the 2026 AprilTag field layout
- publishes a `Field2d` object
- exposes field-oriented drive helpers
- configures PathPlanner `AutoBuilder`
- returns `PathPlannerAuto` instances by name
- fuses Limelight pose estimates into odometry during `periodic()`
- provides `alignToOffset()`, which pathfinds to a tag-relative field pose using MegaTag2 estimates

Current limitations:

- vision fusion only checks `tagCount > 0`
- `periodic()` prints aggressively every cycle, which will spam logs on real hardware

### `src/main/java/frc/robot/subsystems/VisionSubsystem.java`

This is a thin cache over `LimelightHelpers`. It updates:

- target valid flag
- closest AprilTag ID
- `tx`
- `ty`

It keeps command code simple by exposing cached getters instead of raw NetworkTables calls.

### `src/main/java/frc/robot/subsystems/intake.java`

The file name is lowercase, but the public class is `Intake`. The subsystem owns:

- roller SparkMax, CAN ID `9`
- extension SparkMax, CAN ID `10`

It uses a small `PeriodicIO` struct to stage demand/current values and exposes percent-output setters plus stop helpers. The case mismatch is a portability risk on case-sensitive filesystems.

### `src/main/java/frc/robot/subsystems/Indexer.java`

`Indexer` follows the same pattern as `Intake`:

- one SparkMax indexer motor, CAN ID `11`
- `PeriodicIO` state holder
- percent-output control with a simple `stop()`

### `src/main/java/frc/robot/subsystems/Turret.java`

`Turret` is the most feature-rich mechanism subsystem. It manages:

- flywheel leader, CAN ID `14`
- flywheel follower, CAN ID `15`
- hood motor, CAN ID `16`
- turret rotation motor, CAN ID `17`
- encoder-backed hood and rotation feedback
- PID-based hood and rotation control modes
- soft limits for hood and rotation travel
- SmartDashboard telemetry

This is the subsystem used by both manual superstructure commands and the automated shooting commands.

### `src/main/java/frc/robot/subsystems/ExampleSubsystem.java`

This is unused WPILib starter code.

## Vision Helper Layer

`src/main/java/frc/robot/LimelightHelpers.java` is the large static utility that wraps Limelight NetworkTables and pose-estimation APIs. The current codebase uses it for:

- `getTV`, `getTX`, `getTY`
- `getFiducialID`
- `getBotPose_TargetSpace`
- `getBotPoseEstimate_wpiBlue`
- `getBotPoseEstimate_wpiBlue_MegaTag2`

`VisionSubsystem` uses it to cache target data, and `SwerveSubsystem` uses it to inject vision measurements into odometry.

## Autonomous and Deploy Assets

### Autonomous sequence in code

`RobotContainer.getAutonomousCommand()` currently builds:

1. `drivebase.getAutonomousCommand("ScoreLeftSideAuto")`
2. `ShootAuto.shoot(turret, indexer, vision)`

The intent is to drive a PathPlanner routine first and then run an autonomous shooting sequence.

### Alliance-specific shooting targets

`RobotContainer` defines hardcoded field targets used by shooting logic:

- blue alliance target: `(0.0, 5.5)`
- red alliance target: `(16.54, 5.5)`

### PathPlanner assets

Files under `src/main/deploy/pathplanner/autos`:

- `ScoreLeftSideAuto.auto`
- `ScoreRightSideAuto.auto`
- `ScoreIntakeScore.auto`
- `Simple Auto.auto`

Files under `src/main/deploy/pathplanner/paths`:

- `ScoreLeftSide.path`
- `ScoreRightSide.path`
- `IntakeBack.path`
- `IntakeToShoot.path`
- `Simple Path.path`

`ScoreIntakeScore.auto` is the most involved asset. It chains:

1. `ScoreLeftSide`
2. wait `3.0` seconds
3. `IntakeBack`
4. wait `3.0` seconds
5. `IntakeToShoot`

### Swerve deploy config

Files under `src/main/deploy/swerve`:

- `swervedrive.json`
- `controllerproperties.json`
- `modules/frontleft.json`
- `modules/frontright.json`
- `modules/backleft.json`
- `modules/backright.json`
- `modules/physicalproperties.json`
- `modules/pidfproperties.json`

`swervedrive.json` defines:

- `pigeon2` IMU on CAN ID `17`
- four module config files

Module JSONs define the actual drivetrain hardware map:

- `frontleft`: drive `4`, angle `3`, CANcoder `21`, offset `31.20`
- `frontright`: drive `2`, angle `1`, CANcoder `22`, offset `59.50`
- `backleft`: drive `8`, angle `7`, CANcoder `24`, offset `100.53`
- `backright`: drive `6`, angle `5`, CANcoder `23`, offset `297.68`

That mapping is what `SwerveSubsystem` loads through `SwerveParser` at runtime.

## Configuration and Tooling

### Gradle and WPILib

- `build.gradle` uses the `edu.wpi.first.GradleRIO` plugin at `2026.2.1`
- source and target compatibility are both Java 17
- desktop simulation is enabled
- JUnit 5 is configured for tests
- deploy copies `src/main/deploy` to `/home/lvuser/deploy`

`settings.gradle` points Gradle at the local WPILib Maven cache.

### Team and IDE config

- `.wpilib/wpilib_preferences.json` sets team `5190` and language `java`
- `.vscode/launch.json` contains WPILib desktop and roboRIO debug targets
- `.vscode/settings.json` enables automatic Java build config updates and configures the WPILib unit-test environment

### Vendor dependencies

`vendordeps/` contains JSON descriptors for:

- `AdvantageKit`
- `PathplannerLib`
- `Phoenix5`
- `Phoenix6`
- `REVLib`
- `ReduxLib`
- `Studica`
- `StudicaLib`
- `ThriftyLib`
- `WPILibNewCommands`
- `yagsl`

### Global constants

`src/main/java/frc/robot/Constants.java` currently centralizes only a few values:

- driver controller port `0`
- driver deadband `0.05`
- `maximumSpeed = Units.feetToMeters(1)` (~`0.305 m/s`)

Most hardware IDs and tuning values still live inside subsystem-local `Constants` inner classes.

## Repository Layout

```text
src/main/java/frc/robot/
├── Main.java
├── Robot.java
├── RobotContainer.java
├── Constants.java
├── Superstructure.java
├── LimelightHelpers.java
├── commands/
│   ├── AlignTurretToTag.java
│   ├── Autos.java
│   ├── ExampleCommand.java
│   ├── ShootAuto.java
│   └── ShootOnTheMove.java
└── subsystems/
    ├── ExampleSubsystem.java
    ├── Indexer.java
    ├── SwerveSubsystem.java
    ├── Turret.java
    ├── VisionSubsystem.java
    └── intake.java

src/main/deploy/
├── pathplanner/
│   ├── autos/
│   └── paths/
└── swerve/
    ├── controllerproperties.json
    ├── swervedrive.json
    └── modules/

vendordeps/
.wpilib/
.vscode/
```

## Known Risks and Cleanup Targets

- Repair `ShootAuto.java` so autonomous compilation and runtime flow match `RobotContainer`.
- Resolve the controller-port mismatch between `Constants.java` and `RobotContainer`.
- Verify CAN ID `17` is not double-booked between the turret rotation motor and the Pigeon2.
- Replace placeholder drivetrain and shooting tuning values, especially `maximumSpeed`.
- Remove or clearly isolate WPILib template files.
- Reduce `System.out.println` spam in `SwerveSubsystem.periodic()`.

## Summary

The repo already has the shape of a real competition robot codebase: a command-scheduled WPILib runtime, swerve driving, Limelight-backed localization, a turret subsystem, and PathPlanner auto assets. The most mature active behavior is the swerve + `ShootOnTheMove` path. The main missing step is cleanup and stabilization so the autonomous and hardware configuration match the intended design.

Team 5190's 2026 robot codebase is a Java 17 WPILib command-based project built on GradleRIO, YAGSL swerve, PathPlanner autos, REV motor controllers, CTRE sensors, and Limelight AprilTag vision. The code currently centers on a swerve drivetrain, a Limelight-backed targeting stack, and a scoring superstructure made up of an intake, indexer, and turret.

## Repository Layout

- `src/main/java/frc/robot/Main.java` boots WPILib and starts `Robot`.
- `src/main/java/frc/robot/Robot.java` owns mode transitions and runs the `CommandScheduler`.
- `src/main/java/frc/robot/RobotContainer.java` wires subsystems, controller bindings, default drive behavior, and autonomous selection.
- `src/main/java/frc/robot/subsystems/` contains hardware-facing code for the swerve, vision, intake, indexer, and turret.
- `src/main/java/frc/robot/commands/` contains targeting and shooting commands.
- `src/main/deploy/pathplanner/` stores `.auto` and `.path` assets used by PathPlanner.
- `src/main/deploy/swerve/` stores the YAGSL swerve JSON configuration loaded at runtime.
- `vendordeps/` tracks external robotics libraries such as YAGSL, PathPlanner, REVLib, Phoenix 5/6, ThriftyLib, AdvantageKit, ReduxLib, and WPILib command support.

## Robot Lifecycle and Wiring

The startup path is standard WPILib. `Main` calls `RobotBase.startRobot(Robot::new)`, and `Robot` constructs a single `RobotContainer`. Every scheduler tick, `robotPeriodic()` runs `CommandScheduler.getInstance().run()`, which drives all command-based behavior.

`RobotContainer` is the real wiring hub. It creates:

- `SwerveSubsystem drivebase`
- `VisionSubsystem vision`
- `Intake intake`
- `Indexer indexer`
- `Turret turret`
- `Superstructure superstructure`

It also defines alliance-specific scoring targets at `(0.0, 5.5)` for blue and `(16.54, 5.5)` for red, which are consumed by `ShootOnTheMove`.

The default drive command is a field-oriented `SwerveInputStream` built from the Xbox controller:

- left stick Y/X drive translation
- right stick X drives angular velocity
- 0.05 deadband from `Constants.OperatorConstants.DEADBAND`
- translation scaled to 80%
- alliance-relative control enabled

There is also an unused direct-angle drive stream that derives heading from the right stick X/Y pair.

## Operator Controls

The current driver bindings live in `RobotContainer.configureBindings()`:

- `A`: run intake roller forward at `0.6`
- `B`: run intake roller backward at `-0.6`
- `Right Bumper`: extend intake at `0.2`
- `Left Bumper`: retract intake at `-0.2`
- `Left Trigger`: run `ShootOnTheMove`

Commented-out bindings show intended future use for indexer jogging and AprilTag-relative drive alignment. The command-based example trigger still exists, but it only points at the WPILib template subsystem and command.

## Subsystems and How They Work

### SwerveSubsystem

`SwerveSubsystem` loads its hardware definition from `src/main/deploy/swerve/` using `new SwerveParser(directory).createSwerveDrive(...)`. It sets telemetry to `HIGH`, seeds the robot pose to `(1 m, 4 m, 0 deg)`, loads the 2026 welded field AprilTag layout, and publishes a `Field2d` to SmartDashboard.

In `periodic()`, it asks `LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight")` for a pose estimate. If a tag is visible, the result is fused into the swerve odometry with `addVisionMeasurement(...)`. The subsystem also exposes:

- `driveFieldOriented(...)` for field-relative control
- `setUpPathPlanner()` to configure `AutoBuilder`
- `getAutonomousCommand(pathName)` to return a `PathPlannerAuto`
- `seesTag(tagId)` for quick AprilTag checks
- `alignToOffset(...)` to pathfind to a field pose derived from a tag-relative offset

### VisionSubsystem and LimelightHelpers

`VisionSubsystem` is a thin cache over Limelight data. Each loop it stores:

- `tx` horizontal offset
- `ty` vertical offset
- `closestTagID`
- `hasTag`

`LimelightHelpers.java` is the large helper library that wraps NetworkTables access, pose estimates, target-space transforms, and AprilTag metadata. The rest of the code treats it as the only direct API into the Limelight.

### Intake and Indexer

`Intake` controls two Spark MAX motors:

- roller motor CAN ID `9`
- extension motor CAN ID `10`

It stores current draw and percent demands in a `PeriodicIO` struct, then writes the demands in `periodic()`.

`Indexer` is a single Spark MAX on CAN ID `11` and follows the same `PeriodicIO` pattern.

### Turret

`Turret` manages four mechanisms:

- flywheel leader CAN ID `14`
- flywheel follower CAN ID `15`
- hood CAN ID `16`
- rotation CAN ID `17`

The flywheel runs open-loop. Hood and rotation each support manual percent output and PID position control. Soft limits clamp hood travel to `0.0..20.0` encoder rotations and turret rotation to `-10.0..10.0`. SmartDashboard gets live output, position, target, and at-setpoint values every loop.

### Superstructure

`Superstructure` is a command factory layered on top of `Intake`, `Indexer`, and `Turret`. It does not own hardware itself; instead it returns reusable WPILib commands such as:

- `jogRoller`
- `jogExtension`
- `jogIndexer`
- `runFlywheel`
- `jogHoodUp` / `jogHoodDown`
- `setHoodPosition`
- `jogRotationLeft` / `jogRotationRight`
- `setRotationPosition`
- `autoAim`

This keeps low-level motor control in the subsystems and higher-level operator actions in commands.

## Commands and Autonomous Behavior

### Targeting and Shooting Commands

- `AlignTurretToTag` uses Limelight `tx` as a proportional error term. If no tag is visible, it slowly spins the turret to search. It finishes once the horizontal error is under `1.0` degree.
- `ShootOnTheMove` is the most advanced routine in the repository. It reads field-relative chassis speed, projects the robot's future position using a `0.15 s` latency term, estimates shot distance from either Limelight target-space data or odometry, looks up flywheel output from an interpolating distance table, compensates for robot motion with vector subtraction, solves for a hood pitch, and continuously commands turret rotation, hood position, and flywheel output.
- `ShootAuto` is intended to align to a tag, spin up the flywheel, feed with the indexer, and stop the motors.

### Autonomous Selection

`Robot.autonomousInit()` asks `RobotContainer` for an autonomous command and schedules it. Right now `RobotContainer.getAutonomousCommand()` builds:

1. `drivebase.getAutonomousCommand("ScoreLeftSideAuto")`
2. `ShootAuto.shoot(turret, indexer, vision)`

That means the default autonomous plan is: drive the `ScoreLeftSideAuto` PathPlanner routine, then perform an autonomous shot sequence.

### PathPlanner Assets

The checked-in autos are:

- `ScoreLeftSideAuto.auto`: drives `ScoreLeftSide.path`
- `ScoreRightSideAuto.auto`: drives `ScoreRightSide.path`
- `Simple Auto.auto`: drives `Simple Path.path`
- `ScoreIntakeScore.auto`: drives `ScoreLeftSide`, waits 3 seconds, drives `IntakeBack`, waits 3 seconds, then drives `IntakeToShoot`

The path files describe the actual field trajectories. `ScoreLeftSide` and `IntakeToShoot` both use a `ScorePosition` waypoint, while `IntakeBack` and `IntakeToShoot` also use an `IntakePosition` waypoint, which makes the intended cycle path easy to identify.

## Swerve and Hardware Configuration

The YAGSL config in `src/main/deploy/swerve/` defines:

- IMU: Pigeon2 on CAN ID `17`
- front-left module: drive `4`, angle `3`, CANcoder `21`, offset `31.20`
- front-right module: drive `2`, angle `1`, CANcoder `22`, offset `59.50`
- back-left module: drive `8`, angle `7`, CANcoder `24`, offset `100.53`
- back-right module: drive `6`, angle `5`, CANcoder `23`, offset `297.68`

Physical and control properties are also externalized:

- robot mass `35 kg`
- wheel diameter `4 in`
- drive gear ratio `6.12`
- angle gear ratio `21.42`
- current limits `40 A` drive / `20 A` angle
- ramp rate `0.25`
- heading controller gains `P=0.4`, `D=0.01`

Keeping those values in deploy JSON means drive tuning can change without editing Java code.

## Build, Run, and Deploy

Use the WPILib Java 17 toolchain. The project targets GradleRIO `2026.2.1`.

Common commands:

```bash
bash gradlew build
bash gradlew test
bash gradlew simulateJava
bash gradlew deploy
```

Notes:

- the wrapper is committed without the executable bit, so `bash gradlew ...` is the safest invocation
- `.wpilib/wpilib_preferences.json` sets the team number to `5190`
- `.vscode/launch.json` includes both desktop and roboRIO WPILib debug targets
- `.vscode/settings.json` enables automatic Java build updates and configures WPILib unit-test runtime paths

## Current Risks and Rough Edges

This repository has several active development issues that are worth knowing before building or deploying:

- `ShootAuto.java` is incomplete as committed: it references `vision` without accepting it in the method signature, misspells `withTimeout`, and is missing the closing syntax needed to compile cleanly.
- `RobotContainer` creates the Xbox controller on port `1`, but `Constants.OperatorConstants.kDriverControllerPort` is `0`.
- `Turret.Constants.kRotationId` and the Pigeon2 IMU in `swervedrive.json` both use CAN ID `17`, which is a hardware conflict if both devices are present on the same bus.
- `src/main/java/frc/robot/subsystems/intake.java` declares `public class Intake`; that case mismatch can break builds on case-sensitive filesystems.
- `Constants.maximumSpeed` is set to `Units.feetToMeters(1)`, which is only about `0.3048 m/s` and likely a placeholder rather than a final competition speed.
- `SwerveSubsystem.periodic()` prints pose and vision data every scheduler loop, which will flood logs during runtime.
- `ExampleSubsystem`, `ExampleCommand`, and `Autos` are still WPILib template leftovers and should not be mistaken for production robot behavior.

## Summary

The codebase already has a clear architecture: `Robot` runs WPILib, `RobotContainer` wires the machine together, the subsystems own motors and sensors, and the commands compose those pieces into drive, alignment, and shooting behavior. The most developed systems today are the YAGSL swerve stack, the Limelight-assisted targeting path, and the intake/indexer/turret superstructure around scoring. The main work left is hardening the autonomous shot path, cleaning up placeholder code, and reconciling the configuration mismatches called out above.
