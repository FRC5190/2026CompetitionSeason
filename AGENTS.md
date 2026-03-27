# Repository Guidelines

## Project Structure & Runtime Wiring
`src/main/java/frc/robot` contains the WPILib entry points plus the project-specific wiring and support code: `Main`, `Robot`, `RobotContainer`, `Constants`, `Superstructure`, `DashboardPublisher`, and `LimelightHelpers`.

`Robot` is intentionally thin. `RobotContainer` is the composition root for the live robot and currently instantiates `SwerveSubsystem`, `VisionSubsystem`, `Intake`, `Indexer`, `Turret`, and `Superstructure`. `Superstructure` is not a subsystem; it is a command factory that wraps intake, indexer, and turret actions into reusable `Command`s.

Put mechanism implementations in `src/main/java/frc/robot/subsystems` and standalone command classes in `src/main/java/frc/robot/commands`, but match the existing style: simple actions can live as command factories on `Superstructure` or `SwerveSubsystem` when that keeps wiring clearer than adding another class. Template WPILib files are still present (`ExampleSubsystem`, `ExampleCommand`, `Autos`), so do not assume everything in-tree is production code.

## Deploy Assets & Config
`src/main/deploy` is runtime-critical, not just deploy packaging. `Robot.robotInit()` serves the deploy directory over the built-in web server on port `5800`, and `SwerveSubsystem` loads YAGSL config from `src/main/deploy/swerve` at startup.

`src/main/deploy/pathplanner` holds the committed `navgrid.json`, `settings.json`, four `.auto` files, and five `.path` files. Keep PathPlanner assets, chooser names in `RobotContainer`, and autonomous code paths in sync. The chooser exists today, but `RobotContainer.getAutonomousCommand()` currently returns only a `ShootAuto` sequence named after the selected option, so autonomous selection is only partially wired.

`src/main/deploy/swerve` contains the full YAGSL JSON set: `swervedrive.json`, controller properties, shared PIDF and physical properties, and four per-module configs. `src/main/deploy/elastic-layout.json` depends on the SmartDashboard topics published by `DashboardPublisher`, so update both files together when renaming or restructuring dashboard telemetry. Keep `Hardware.md` updated whenever CAN IDs, controllers, or deploy JSON change.

Third-party descriptors live in `vendordeps/`. This repo currently depends on WPILib commands, YAGSL, PathPlannerLib, REVLib, Phoenix5, Phoenix6, ThriftyLib, AdvantageKit, ReduxLib, and Studica libraries, so do not remove vendordeps casually.

## Build, Test, and Development Commands
Use the WPILib Java 17 toolchain. `build.gradle` targets GradleRIO `2026.2.1` with `sourceCompatibility` and `targetCompatibility` set to Java 17. In the current local environment, `bash gradlew test` fails under Java 25 while Gradle is creating the `test` task, so switch to Java 17 before judging the repo’s build health.

`bash gradlew build` compiles the robot code and packages deploy assets.
`bash gradlew test` runs JUnit 5 tests once the project is on Java 17.
`bash gradlew simulateJava` launches the desktop simulator.
`bash gradlew deploy` builds and deploys to team `5190` using `.wpilib/wpilib_preferences.json`.

The wrapper has not been reliable as an executable across machines, so prefer `bash gradlew ...`.

## Coding Style & Naming Conventions
Stay in `frc.robot` and follow the existing Java style: 2-space indentation, `UpperCamelCase` classes, `camelCase` methods, and `kConstantName` for constants. One public class per file still applies.

Match the repo’s field naming before “cleaning it up.” Many private members use trailing underscores such as `turret_`, `drivebase_`, and `io_`. Subsystem-specific constants often live in nested `Constants` classes inside each subsystem instead of only in `frc.robot.Constants`.

Keep orchestration readable. `RobotContainer` should stay focused on composition and bindings, `Superstructure` should keep encapsulating multi-subsystem command snippets, and subsystem APIs should stay narrow. If you touch autonomous behavior, inspect both `RobotContainer` and `SwerveSubsystem` so chooser wiring, PathPlanner assets, and YAGSL integration remain consistent.

## Testing & Validation
There is currently no committed `src/test/java` tree. Add new tests under `src/test/java/frc/robot/...` with names like `ShootAutoTest.java` or `TurretTest.java`.

Prioritize unit tests for math-heavy or command-heavy code such as `ShootOnTheMove`, `ShootAuto`, `Superstructure`, and target-state transitions. For hardware-bound changes such as CAN IDs, Limelight behavior, dashboard keys, PathPlanner files, or swerve JSON, include manual validation notes from sim or on-robot testing.

When changing dashboard output, verify both `DashboardPublisher` and `src/main/deploy/elastic-layout.json`. When changing drivetrain or autonomous config, check for drift between `pathplanner/settings.json`, `swerve/modules/physicalproperties.json`, and the code that assumes those values.

## Commit & Pull Request Guidelines
Recent history is mixed, but the clearest pattern is short imperative commits, often with a scope prefix such as `feat:` or `fix:`. Prefer concise, subsystem-specific subjects like `fix: stop hood after auto shot` or `feat: add dashboard auto chooser`.

PRs should summarize the robot behavior change, list touched subsystems and deploy/config files, and state how the change was validated. Include screenshots or sim output for dashboard or autonomous changes, and mention any remaining field-tuning work.
