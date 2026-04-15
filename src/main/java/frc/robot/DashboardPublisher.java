// package frc.robot;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Turret;

// public final class DashboardPublisher {
//   private static final double kDemandDeadband = 0.10;
//   private static final double kMotorFreeSpeedRpm = 5676.0;
//   private static final double kFlywheelReadyFraction = 0.85;
//   private static final double kIndexerReadyFraction = 0.50;
//   private static final double kIndexerJamCurrentAmps = 25.0;
//   private static final double kIndexerJamDebounceSec = 0.25;

//   private static final String kStatusGreen = "#2ecc71";
//   private static final String kStatusYellow = "#f1c40f";

//   private final RobotContainer robotContainer_;
//   private double indexerJamStartSec_ = Double.NaN;

//   public DashboardPublisher(RobotContainer robotContainer) {
//     robotContainer_ = robotContainer;
//     publishSendables();
//   }

//   private void publishSendables() {
//     SmartDashboard.putData("Field", robotContainer_.drivebase.getField());
//     SmartDashboard.putData("Autos/Chooser", robotContainer_.getAutonomousChooser());
//   }

//   public void update() {
//     Pose2d pose = robotContainer_.drivebase.getSwerveDrive().getPose();
//     ChassisSpeeds robotVelocity = robotContainer_.drivebase.getSwerveDrive().getRobotVelocity();
//     double batteryVoltage = RobotController.getBatteryVoltage();

//     boolean flywheelReady = isFlywheelReady();
//     boolean hoodReady = robotContainer_.turret.isHoodNearTarget();
//     boolean intakeReady = robotContainer_.intake.isExtensionNearTarget();
//     boolean indexerReady = isIndexerReady();
//     boolean indexerJammed = updateIndexerJamState(indexerReady);

//     SmartDashboard.putString("Robot/Mode", getRobotMode());
//     SmartDashboard.putBoolean("Robot/Enabled", DriverStation.isEnabled());
//     SmartDashboard.putString("Robot/Alliance", getAllianceName());
//     SmartDashboard.putNumber("Robot/BatteryVoltage", batteryVoltage);
//     SmartDashboard.putBoolean("Robot/Brownout", RobotController.isBrownedOut());
//     SmartDashboard.putNumber("Robot/MatchTime", DriverStation.getMatchTime());

//     SmartDashboard.putNumber("Drive/PoseX", pose.getX());
//     SmartDashboard.putNumber("Drive/PoseY", pose.getY());
//     SmartDashboard.putNumber("Drive/HeadingDeg", pose.getRotation().getDegrees());
//     SmartDashboard.putNumber("Drive/LinearSpeedMps",
//         Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));
//     SmartDashboard.putNumber("Drive/AngularSpeedDegPerSec",
//         Math.toDegrees(robotVelocity.omegaRadiansPerSecond));

//     SmartDashboard.putBoolean("Vision/HasTag", robotContainer_.vision.hasTag());
//     SmartDashboard.putNumber("Vision/TagId", robotContainer_.vision.getClosestTagID());
//     SmartDashboard.putNumber("Vision/TxDeg", robotContainer_.vision.getTX());
//     SmartDashboard.putNumber("Vision/TyDeg", robotContainer_.vision.getTY());
//     SmartDashboard.putNumber("Vision/TargetDistanceM",
//         robotContainer_.vision.getTargetDistanceMeters());

//     SmartDashboard.putNumber("Turret/FlywheelDemand", robotContainer_.turret.getFlywheelDemand());
//     SmartDashboard.putNumber("Turret/FlywheelOutput", robotContainer_.turret.getFlywheelPercent());
//     SmartDashboard.putNumber("Turret/FlywheelLeaderVelocityRpm",
//         robotContainer_.turret.getFlywheelLeaderVelocity());
//     SmartDashboard.putNumber("Turret/FlywheelFollowerVelocityRpm",
//         robotContainer_.turret.getFlywheelFollowerVelocity());
//     SmartDashboard.putNumber("Turret/FlywheelAverageVelocityRpm",
//         robotContainer_.turret.getFlywheelAverageVelocityRpm());
//     SmartDashboard.putBoolean("Turret/FlywheelReady", flywheelReady);
//     SmartDashboard.putString("Turret/FlywheelStatusText", getFlywheelStatusText(flywheelReady));
//     SmartDashboard.putString("Turret/FlywheelStatusColor", getStatusColor(flywheelReady));

//     SmartDashboard.putNumber("Turret/HoodPosition", robotContainer_.turret.getHoodPosition());
//     SmartDashboard.putNumber("Turret/HoodTarget", robotContainer_.turret.getHoodTarget());
//     SmartDashboard.putBoolean("Turret/HoodReady", hoodReady);
//     SmartDashboard.putString("Turret/HoodDirectionText", getHoodDirectionText());
//     SmartDashboard.putString("Turret/HoodStatusColor", getStatusColor(hoodReady));

//     SmartDashboard.putNumber("Indexer/Demand", robotContainer_.indexer.getIndexerPercent());
//     SmartDashboard.putNumber("Indexer/VelocityRpm", robotContainer_.indexer.getLeaderVelocityRpm());
//     SmartDashboard.putNumber("Indexer/LeaderCurrent", robotContainer_.indexer.getLeaderCurrent());
//     SmartDashboard.putNumber("Indexer/FollowerCurrent", robotContainer_.indexer.getFollowerCurrent());
//     SmartDashboard.putBoolean("Indexer/Ready", indexerReady);
//     SmartDashboard.putBoolean("Indexer/Jammed", indexerJammed);
//     SmartDashboard.putString("Indexer/ReadyText", getIndexerReadyText(indexerReady));
//     SmartDashboard.putString("Indexer/ReadyColor", getStatusColor(indexerReady));
//     SmartDashboard.putString("Indexer/JamText", indexerJammed ? "Jammed" : "Clear");
//     SmartDashboard.putString("Indexer/JamColor", getStatusColor(!indexerJammed));

//     SmartDashboard.putNumber("Intake/RollerDemand", robotContainer_.intake.getRollerDemand());
//     SmartDashboard.putNumber("Intake/RollerCurrent", robotContainer_.intake.getRollerCurrent());
//     SmartDashboard.putNumber("Intake/RollerVelocity", robotContainer_.intake.getRollerVelocity());
//     SmartDashboard.putNumber("Intake/ExtensionDemand", robotContainer_.intake.getExtensionDemand());
//     SmartDashboard.putNumber("Intake/ExtensionPosition", robotContainer_.intake.getExtensionPosition());
//     SmartDashboard.putNumber("Intake/ExtensionTarget", robotContainer_.intake.getExtensionTarget());
//     SmartDashboard.putBoolean("Intake/ExtensionAtTarget", intakeReady);
//     SmartDashboard.putNumber("Intake/ExtensionCurrent", robotContainer_.intake.getExtensionCurrent());
//     SmartDashboard.putString("Intake/StatusText", getIntakeStatusText());
//     SmartDashboard.putString("Intake/StatusColor", getStatusColor(intakeReady));

//     String selectedAuto = robotContainer_.getSelectedAutoName();
//     boolean selectedAutoValid = selectedAuto != null && !selectedAuto.isBlank();
//     SmartDashboard.putString("Autos/SelectedName",
//         selectedAutoValid ? selectedAuto : robotContainer_.getResolvedAutoName());
//     SmartDashboard.putBoolean("Autos/SelectedValid", selectedAutoValid);
//   }

//   private boolean isFlywheelReady() {
//     double demand = Math.abs(robotContainer_.turret.getFlywheelDemand());
//     if (demand < kDemandDeadband) {
//       return false;
//     }
//     double expectedRpm = demand * kMotorFreeSpeedRpm;
//     return robotContainer_.turret.getFlywheelAverageVelocityRpm()
//         >= expectedRpm * kFlywheelReadyFraction;
//   }

//   private boolean isIndexerReady() {
//     double demand = Math.abs(robotContainer_.indexer.getIndexerPercent());
//     if (demand < kDemandDeadband) {
//       return false;
//     }
//     double expectedRpm = demand * kMotorFreeSpeedRpm;
//     return Math.abs(robotContainer_.indexer.getLeaderVelocityRpm())
//         >= expectedRpm * kIndexerReadyFraction;
//   }

//   private boolean updateIndexerJamState(boolean indexerReady) {
//     double demand = Math.abs(robotContainer_.indexer.getIndexerPercent());
//     if (demand < kDemandDeadband) {
//       indexerJamStartSec_ = Double.NaN;
//       return false;
//     }

//     boolean jamCandidate = robotContainer_.indexer.getLeaderCurrent() >= kIndexerJamCurrentAmps
//         && !indexerReady;
//     if (!jamCandidate) {
//       indexerJamStartSec_ = Double.NaN;
//       return false;
//     }

//     double now = Timer.getFPGATimestamp();
//     if (Double.isNaN(indexerJamStartSec_)) {
//       indexerJamStartSec_ = now;
//       return false;
//     }
//     return (now - indexerJamStartSec_) >= kIndexerJamDebounceSec;
//   }

//   private String getFlywheelStatusText(boolean flywheelReady) {
//     if (Math.abs(robotContainer_.turret.getFlywheelDemand()) < kDemandDeadband) {
//       return "Idle";
//     }
//     return flywheelReady ? "At Speed" : "Spinning Up";
//   }

//   private String getHoodDirectionText() {
//     double hoodOutput = robotContainer_.turret.getHoodPercent();
//     if (Math.abs(hoodOutput) > 0.02) {
//       return hoodOutput > 0.0 ? "Up" : "Down";
//     }

//     double hoodError =
//         robotContainer_.turret.getHoodTarget() - robotContainer_.turret.getHoodPosition();
//     if (Math.abs(hoodError) <= Turret.Constants.kHoodTolerance) {
//       return "Holding";
//     }
//     return hoodError > 0.0 ? "Up" : "Down";
//   }

//   private String getIndexerReadyText(boolean indexerReady) {
//     if (Math.abs(robotContainer_.indexer.getIndexerPercent()) < kDemandDeadband) {
//       return "Idle";
//     }
//     return indexerReady ? "Ready" : "Feeding";
//   }

//   private String getIntakeStatusText() {
//     double extensionDemand = robotContainer_.intake.getExtensionDemand();
//     if (Math.abs(extensionDemand) > 0.02) {
//       return extensionDemand > 0.0 ? "Extending" : "Retracting";
//     }

//     double extensionError =
//         robotContainer_.intake.getExtensionTarget() - robotContainer_.intake.getExtensionPosition();
//     if (Math.abs(extensionError) <= Intake.Constants.kExtensionTolerance) {
//       return "At Target";
//     }
//     return extensionError > 0.0 ? "Extending" : "Retracting";
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

//   private String getStatusColor(boolean ready) {
//     return ready ? kStatusGreen : kStatusYellow;
//   }
// }
