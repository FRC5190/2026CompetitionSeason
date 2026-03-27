// package frc.robot;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.Intake;

// public final class DashboardPublisher {
//   private static final double kLowBatteryWarningVolts = 11.5;

//   private final RobotContainer robotContainer_;
//   private final Alert lowBatteryAlert_ = new Alert("Battery voltage is low.", AlertType.kWarning);
//   private final Alert visionAlert_ =
//       new Alert("Limelight does not have a valid AprilTag target.", AlertType.kInfo);
//   private final Alert autoSelectionAlert_ =
//       new Alert("No autonomous routine is selected.", AlertType.kError);

//   public DashboardPublisher(RobotContainer robotContainer) {
//     robotContainer_ = robotContainer;
//     publishSendables();
//     publishDashboardCommands();
//   }

//   private void publishSendables() {
//     SmartDashboard.putData("Field", robotContainer_.drivebase.getField());
//     SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
//     SmartDashboard.putData("Autos/Chooser", robotContainer_.getAutonomousChooser());

//     SmartDashboard.putData("Subsystems/Drivebase", robotContainer_.drivebase);
//     SmartDashboard.putData("Subsystems/Vision", robotContainer_.vision);
//     SmartDashboard.putData("Subsystems/Intake", robotContainer_.intake);
//     SmartDashboard.putData("Subsystems/Indexer", robotContainer_.indexer);
//     SmartDashboard.putData("Subsystems/Turret", robotContainer_.turret);
//   }

//   private void publishDashboardCommands() {
//     // publishCommand("Dashboard/Commands/ExtendIntake", "Extend Intake",
//     // robotContainer_.superstructure
//     // .setExtensionPosition(Intake.Constants.kMaxExtensionPosition));
//     // publishCommand("Dashboard/Commands/RetractIntake", "Retract Intake",
//     // robotContainer_.superstructure
//     // .setExtensionPosition(Intake.Constants.kMinExtensionPosition));
//     publishCommand("Dashboard/Commands/IntakeForwardBurst", "Intake Forward Burst",
//         Commands
//             .startEnd(() -> robotContainer_.intake.setRollerPercent(0.6),
//                 () -> robotContainer_.intake.stopRoller(), robotContainer_.intake)
//             .withTimeout(1.0));
//     publishCommand("Dashboard/Commands/IntakeReverseBurst", "Intake Reverse Burst",
//         Commands
//             .startEnd(() -> robotContainer_.intake.setRollerPercent(-0.6),
//                 () -> robotContainer_.intake.stopRoller(), robotContainer_.intake)
//             .withTimeout(1.0));
//     publishCommand("Dashboard/Commands/IndexerFeedBurst", "Indexer Feed Burst",
//         Commands.startEnd(() -> robotContainer_.indexer.setPercent(0.8),
//             () -> robotContainer_.indexer.stop(), robotContainer_.indexer).withTimeout(0.75));
//     publishCommand("Dashboard/Commands/ShooterStow", "Shooter Stow", Commands.run(() -> {
//       robotContainer_.turret.stopFlywheel();
//       robotContainer_.turret.setHoodPosition(0.0);
//       robotContainer_.turret.setRotationPosition(0.0);
//     }, robotContainer_.turret).until(() -> robotContainer_.turret.isHoodAtTarget()
//         && robotContainer_.turret.isRotationAtTarget()).withTimeout(2.0));
//     publishCommand("Dashboard/Commands/StopAllMechanisms", "Stop All Mechanisms",
//         Commands.runOnce(() -> {
//           robotContainer_.intake.stopAll();
//           robotContainer_.indexer.stop();
//           robotContainer_.turret.stopAll();
//         }, robotContainer_.intake, robotContainer_.indexer, robotContainer_.turret));
//   }

//   private void publishCommand(String key, String name, Command command) {
//     SmartDashboard.putData(key, command.withName(name));
//   }

//   public void update() {
//     Pose2d pose = robotContainer_.drivebase.getSwerveDrive().getPose();
//     ChassisSpeeds robotVelocity = robotContainer_.drivebase.getSwerveDrive().getRobotVelocity();
//     double batteryVoltage = RobotController.getBatteryVoltage();

//     SmartDashboard.putString("Robot/Mode", getRobotMode());
//     SmartDashboard.putBoolean("Robot/Enabled", DriverStation.isEnabled());
//     SmartDashboard.putString("Robot/Alliance", getAllianceName());
//     SmartDashboard.putNumber("Robot/BatteryVoltage", batteryVoltage);
//     SmartDashboard.putBoolean("Robot/Brownout", RobotController.isBrownedOut());
//     SmartDashboard.putNumber("Robot/MatchTime", DriverStation.getMatchTime());

//     SmartDashboard.putNumber("Drive/HeadingDeg", pose.getRotation().getDegrees());
//     SmartDashboard.putNumber("Drive/PoseX", pose.getX());
//     SmartDashboard.putNumber("Drive/PoseY", pose.getY());
//     SmartDashboard.putNumber("Drive/LinearSpeedMps",
//         Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));
//     SmartDashboard.putNumber("Drive/AngularSpeedDegPerSec",
//         Math.toDegrees(robotVelocity.omegaRadiansPerSecond));
//     SmartDashboard.putBoolean("Drive/VisionPoseValid",
//         robotContainer_.drivebase.isVisionPoseValid());

//     SmartDashboard.putBoolean("Vision/HasTag", robotContainer_.vision.hasTag());
//     SmartDashboard.putNumber("Vision/TagId", robotContainer_.vision.getClosestTagID());
//     SmartDashboard.putNumber("Vision/TxDeg", robotContainer_.vision.getTX());
//     SmartDashboard.putNumber("Vision/TyDeg", robotContainer_.vision.getTY());
//     SmartDashboard.putNumber("Vision/TargetDistanceM",
//         robotContainer_.vision.getTargetDistanceMeters());

//     SmartDashboard.putNumber("Intake/RollerDemand", robotContainer_.intake.getRollerDemand());
//     SmartDashboard.putNumber("Intake/RollerCurrent", robotContainer_.intake.getRollerCurrent());
//     SmartDashboard.putNumber("Intake/RollerVelocity", robotContainer_.intake.getRollerVelocity());
//     SmartDashboard.putNumber("Intake/ExtensionDemand", robotContainer_.intake.getExtensionDemand());
//     SmartDashboard.putNumber("Intake/ExtensionPosition",
//         robotContainer_.intake.getExtensionPosition());
//     SmartDashboard.putNumber("Intake/ExtensionTarget", robotContainer_.intake.getExtensionTarget());
//     SmartDashboard.putBoolean("Intake/ExtensionAtTarget",
//         robotContainer_.intake.isExtensionAtTarget());
//     SmartDashboard.putNumber("Intake/ExtensionCurrent",
//         robotContainer_.intake.getExtensionCurrent());

//     SmartDashboard.putNumber("Indexer/Demand", robotContainer_.indexer.getIndexerPercent());
//     // SmartDashboard.putNumber("Indexer/Current", robotContainer_.indexer.getIndexerCurrent());

//     SmartDashboard.putNumber("Turret/FlywheelDemand", robotContainer_.turret.getFlywheelDemand());
//     SmartDashboard.putNumber("Turret/FlywheelOutput", robotContainer_.turret.getFlywheelPercent());
//     SmartDashboard.putNumber("Turret/FlywheelLeaderVelocity",
//         robotContainer_.turret.getFlywheelLeaderVelocity());
//     SmartDashboard.putNumber("Turret/FlywheelFollowerVelocity",
//         robotContainer_.turret.getFlywheelFollowerVelocity());
//     SmartDashboard.putNumber("Turret/HoodPosition", robotContainer_.turret.getHoodPosition());
//     SmartDashboard.putNumber("Turret/HoodTarget", robotContainer_.turret.getHoodTarget());
//     SmartDashboard.putBoolean("Turret/HoodAtTarget", robotContainer_.turret.isHoodAtTarget());
//     SmartDashboard.putNumber("Turret/RotationPosition",
//         robotContainer_.turret.getRotationPosition());
//     SmartDashboard.putNumber("Turret/RotationTarget", robotContainer_.turret.getRotationTarget());
//     SmartDashboard.putBoolean("Turret/RotationAtTarget",
//         robotContainer_.turret.isRotationAtTarget());

//     String selectedAuto = robotContainer_.getSelectedAutoName();
//     boolean selectedAutoValid = selectedAuto != null && !selectedAuto.isBlank();
//     SmartDashboard.putString("Autos/SelectedName",
//         selectedAutoValid ? selectedAuto : robotContainer_.getResolvedAutoName());
//     SmartDashboard.putBoolean("Autos/SelectedValid", selectedAutoValid);

//     updateAlerts(batteryVoltage, selectedAutoValid);
//   }

//   private void updateAlerts(double batteryVoltage, boolean selectedAutoValid) {
//     lowBatteryAlert_.setText(String.format("Battery voltage is low: %.1fV", batteryVoltage));
//     lowBatteryAlert_.set(batteryVoltage <= kLowBatteryWarningVolts);

//     visionAlert_.set(DriverStation.isEnabled() && !robotContainer_.vision.hasTag());
//     autoSelectionAlert_.set(!selectedAutoValid);
//   }

//   private String getRobotMode() {
//     if (DriverStation.isTestEnabled()) {
//       return "Test";
//     }
//     if (DriverStation.isAutonomousEnabled()) {
//       return "Autonomous";
//     }
//     if (DriverStation.isTeleopEnabled()) {
//       return "Teleop";
//     }
//     if (DriverStation.isDisabled()) {
//       return "Disabled";
//     }
//     return "Unknown";
//   }

//   private String getAllianceName() {
//     return DriverStation.getAlliance().map(alliance -> switch (alliance) {
//       case Red -> "Red";
//       case Blue -> "Blue";
//     }).orElse("Unknown");
//   }
// }
