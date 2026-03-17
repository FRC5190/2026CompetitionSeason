# Repository Guidelines

## Project Structure & Module Organization
`src/main/java/frc/robot` contains the robot entry points and shared wiring, including `Robot`, `RobotContainer`, `Constants`, and `Superstructure`. Put subsystem implementations in `src/main/java/frc/robot/subsystems` and command logic in `src/main/java/frc/robot/commands`.

`src/main/deploy/pathplanner` stores PathPlanner `.auto` and `.path` files. `src/main/deploy/swerve` holds the YAGSL/swerve JSON configuration that is copied to the roboRIO during deploy. Keep third-party dependency descriptors in `vendordeps/`. Team and IDE metadata live in `.wpilib/` and `.vscode/`.

## Build, Test, and Development Commands
Use the WPILib Java 17 toolchain. This project targets GradleRIO `2026.2.1`; running it under Java 25 currently fails before task execution.

`bash gradlew build` compiles the robot code and packages deploy assets.
`bash gradlew test` runs JUnit 5 tests.
`bash gradlew simulateJava` starts the desktop simulator with WPILib GUI support.
`bash gradlew deploy` builds and deploys to the team 5190 roboRIO using `.wpilib/wpilib_preferences.json`.

The wrapper is committed without the executable bit, so `bash gradlew ...` is the safest cross-machine invocation.

## Coding Style & Naming Conventions
Stay within the command-based WPILib structure and keep packages under `frc.robot`. Follow the existing Java style: two-space indentation, `UpperCamelCase` for classes, `camelCase` for methods and fields, and `kConstantName` or `UPPER_SNAKE_CASE` for constants.

Keep one public class per file, and make the file name match the class name exactly. That matters on case-sensitive environments. Prefer small subsystem APIs and compose behavior in commands or `RobotContainer` rather than burying orchestration inside a subsystem.

## Testing Guidelines
JUnit 5 is configured in [build.gradle](/Users/ayushsagar/Documents/GitHub/2026CompetitionSeason/build.gradle), but there is no committed `src/test/java` tree yet. Add new tests under `src/test/java/frc/robot/...` with names like `TurretTest.java` or `ShootAutoTest.java`.

Unit-test math, command sequencing, and state transitions. For hardware-bound changes such as motor IDs, Limelight tuning, or PathPlanner edits, include brief manual validation notes from sim or robot testing in the PR.

## Commit & Pull Request Guidelines
Recent history favors short imperative subjects such as `working swerve...` and `created pathplanner autos...`. Keep commit titles brief, imperative, and subsystem-specific. Avoid placeholders like `temp`.

PRs should summarize the robot behavior change, list touched subsystems/config paths, and note how the change was validated. Include screenshots or sim output when dashboard behavior, autos, or targeting flows change, and link the related task if one exists.
