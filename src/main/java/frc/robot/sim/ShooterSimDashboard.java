package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public final class ShooterSimDashboard {
  private static final String kRoot = "SimShooter/";

  private final Field2d field_ = new Field2d();
  private final StructArrayPublisher<Pose3d> fuelPosesPublisher_;
  private final StructArrayPublisher<Pose3d> successfulTrajectoryPublisher_;
  private final StructArrayPublisher<Pose3d> missedTrajectoryPublisher_;

  public ShooterSimDashboard() {
    NetworkTable smartDashboardTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    fuelPosesPublisher_ =
        smartDashboardTable.getStructArrayTopic("SimShooter/FuelPoses", Pose3d.struct).publish();
    successfulTrajectoryPublisher_ =
        smartDashboardTable.getStructArrayTopic("SimShooter/Trajectory/Successful", Pose3d.struct)
            .publish();
    missedTrajectoryPublisher_ =
        smartDashboardTable.getStructArrayTopic("SimShooter/Trajectory/Missed", Pose3d.struct).publish();

    SmartDashboard.putData(kRoot + "Field", field_);
  }

  public void publishDefaults() {
    SmartDashboard.putNumber(kRoot + "RobotPoseX", 3.0);
    SmartDashboard.putNumber(kRoot + "RobotPoseY", 4.0);
    SmartDashboard.putNumber(kRoot + "RobotHeadingDeg", 0.0);
    SmartDashboard.putBoolean(kRoot + "ApplyPoseNow", false);
    SmartDashboard.putNumber(kRoot + "RobotVxMps", 0.0);
    SmartDashboard.putNumber(kRoot + "RobotVyMps", 0.0);
    SmartDashboard.putNumber(kRoot + "RobotOmegaDegPerSec", 0.0);
    SmartDashboard.putBoolean(kRoot + "UseIntakeAmmo", false);
    SmartDashboard.putBoolean(kRoot + "IntakeRunning", false);
    SmartDashboard.putBoolean(kRoot + "LoadOneFuel", false);
    SmartDashboard.putBoolean(kRoot + "FireNow", false);
    SmartDashboard.putBoolean(kRoot + "ResetField", false);
    SmartDashboard.putBoolean(kRoot + "ClearProjectiles", false);
    SmartDashboard.putBoolean(kRoot + "RunSweep", false);
    SmartDashboard.putNumber(kRoot + "ShooterRPM", 4200.0);
    SmartDashboard.putNumber(kRoot + "HoodDeg", 35.0);
    SmartDashboard.putNumber(kRoot + "TurretDeg", 0.0);
    SmartDashboard.putNumber(kRoot + "LaunchHeightM", 0.40);
    SmartDashboard.putNumber(kRoot + "ShooterOffsetXM", 0.20);
    SmartDashboard.putNumber(kRoot + "ShooterOffsetYM", 0.0);
    SmartDashboard.putNumber(kRoot + "LaunchSpeedAt6000RpmMps", 20.0);
    SmartDashboard.putNumber(kRoot + "Sweep/RpmMin", 2500.0);
    SmartDashboard.putNumber(kRoot + "Sweep/RpmMax", 6000.0);
    SmartDashboard.putNumber(kRoot + "Sweep/RpmStep", 250.0);
    SmartDashboard.putNumber(kRoot + "Sweep/HoodMinDeg", 20.0);
    SmartDashboard.putNumber(kRoot + "Sweep/HoodMaxDeg", 60.0);
    SmartDashboard.putNumber(kRoot + "Sweep/HoodStepDeg", 2.5);
    SmartDashboard.putString(kRoot + "LastAction", "Ready");
    SmartDashboard.putString(kRoot + "Sweep/Summary", "No sweep run yet");
    SmartDashboard.putString(kRoot + "Sweep/ResultsCsv", "");
    clearTrajectoryPublishers();
    fuelPosesPublisher_.set(new Pose3d[0]);
  }

  public Inputs readInputs() {
    Pose2d requestedPose =
        new Pose2d(
            SmartDashboard.getNumber(kRoot + "RobotPoseX", 3.0),
            SmartDashboard.getNumber(kRoot + "RobotPoseY", 4.0),
            Rotation2d.fromDegrees(SmartDashboard.getNumber(kRoot + "RobotHeadingDeg", 0.0)));

    ChassisSpeeds requestedFieldSpeeds =
        new ChassisSpeeds(
            SmartDashboard.getNumber(kRoot + "RobotVxMps", 0.0),
            SmartDashboard.getNumber(kRoot + "RobotVyMps", 0.0),
            Math.toRadians(SmartDashboard.getNumber(kRoot + "RobotOmegaDegPerSec", 0.0)));

    return new Inputs(
        requestedPose,
        requestedFieldSpeeds,
        SmartDashboard.getNumber(kRoot + "ShooterRPM", 4200.0),
        SmartDashboard.getNumber(kRoot + "HoodDeg", 35.0),
        SmartDashboard.getNumber(kRoot + "TurretDeg", 0.0),
        SmartDashboard.getNumber(kRoot + "LaunchHeightM", 0.40),
        SmartDashboard.getNumber(kRoot + "ShooterOffsetXM", 0.20),
        SmartDashboard.getNumber(kRoot + "ShooterOffsetYM", 0.0),
        SmartDashboard.getNumber(kRoot + "LaunchSpeedAt6000RpmMps", 20.0),
        SmartDashboard.getBoolean(kRoot + "UseIntakeAmmo", false),
        SmartDashboard.getBoolean(kRoot + "IntakeRunning", false),
        consumeBoolean(kRoot + "ApplyPoseNow"),
        consumeBoolean(kRoot + "LoadOneFuel"),
        consumeBoolean(kRoot + "FireNow"),
        consumeBoolean(kRoot + "ResetField"),
        consumeBoolean(kRoot + "ClearProjectiles"),
        consumeBoolean(kRoot + "RunSweep"),
        SmartDashboard.getNumber(kRoot + "Sweep/RpmMin", 2500.0),
        SmartDashboard.getNumber(kRoot + "Sweep/RpmMax", 6000.0),
        SmartDashboard.getNumber(kRoot + "Sweep/RpmStep", 250.0),
        SmartDashboard.getNumber(kRoot + "Sweep/HoodMinDeg", 20.0),
        SmartDashboard.getNumber(kRoot + "Sweep/HoodMaxDeg", 60.0),
        SmartDashboard.getNumber(kRoot + "Sweep/HoodStepDeg", 2.5));
  }

  public void publishRobotPose(Pose2d robotPose, Pose3d hubPose) {
    field_.setRobotPose(robotPose);
    field_.getObject("Hub").setPose(new Pose2d(hubPose.getX(), hubPose.getY(), Rotation2d.kZero));
  }

  public void publishFuelPoses(Pose3d[] fuelPoses) {
    fuelPosesPublisher_.set(fuelPoses);
  }

  public void publishTrajectories(Pose3d[] successfulTrajectory, Pose3d[] missedTrajectory) {
    successfulTrajectoryPublisher_.set(successfulTrajectory);
    missedTrajectoryPublisher_.set(missedTrajectory);
  }

  public void clearTrajectoryPublishers() {
    publishTrajectories(new Pose3d[0], new Pose3d[0]);
  }

  public void publishState(
      Pose2d robotPose,
      int fuelInIntake,
      int fuelOnField,
      int fuelInAir,
      boolean shotActive,
      boolean sweepRunning,
      int sweepCompletedShots,
      int sweepTotalShots,
      Optional<ShotSampleResult> lastShot,
      Optional<ShotSweepResult> lastSweep,
      int allianceScore,
      String lastAction) {
    SmartDashboard.putNumber(kRoot + "State/RobotPoseX", robotPose.getX());
    SmartDashboard.putNumber(kRoot + "State/RobotPoseY", robotPose.getY());
    SmartDashboard.putNumber(kRoot + "State/RobotHeadingDeg", robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber(kRoot + "State/FuelInIntake", fuelInIntake);
    SmartDashboard.putNumber(kRoot + "State/FuelOnField", fuelOnField);
    SmartDashboard.putNumber(kRoot + "State/FuelInAir", fuelInAir);
    SmartDashboard.putBoolean(kRoot + "State/ShotActive", shotActive);
    SmartDashboard.putBoolean(kRoot + "Sweep/Running", sweepRunning);
    SmartDashboard.putNumber(kRoot + "Sweep/CompletedShots", sweepCompletedShots);
    SmartDashboard.putNumber(kRoot + "Sweep/TotalShots", sweepTotalShots);
    SmartDashboard.putNumber(kRoot + "State/AllianceScore", allianceScore);
    SmartDashboard.putString(kRoot + "LastAction", lastAction);

    if (lastShot.isPresent()) {
      ShotSampleResult shot = lastShot.get();
      SmartDashboard.putBoolean(kRoot + "LastShot/Scored", shot.scored());
      SmartDashboard.putNumber(kRoot + "LastShot/RPM", shot.rpm());
      SmartDashboard.putNumber(kRoot + "LastShot/HoodDeg", shot.hoodDeg());
      SmartDashboard.putNumber(kRoot + "LastShot/TurretDeg", shot.turretDeg());
      SmartDashboard.putNumber(kRoot + "LastShot/FlightTimeSec", shot.flightTimeSec());
      SmartDashboard.putNumber(kRoot + "LastShot/TerminalX", shot.terminalPose().getX());
      SmartDashboard.putNumber(kRoot + "LastShot/TerminalY", shot.terminalPose().getY());
      SmartDashboard.putNumber(kRoot + "LastShot/TerminalZ", shot.terminalPose().getZ());
    } else {
      SmartDashboard.putBoolean(kRoot + "LastShot/Scored", false);
      SmartDashboard.putNumber(kRoot + "LastShot/RPM", 0.0);
      SmartDashboard.putNumber(kRoot + "LastShot/HoodDeg", 0.0);
      SmartDashboard.putNumber(kRoot + "LastShot/TurretDeg", 0.0);
      SmartDashboard.putNumber(kRoot + "LastShot/FlightTimeSec", 0.0);
      SmartDashboard.putNumber(kRoot + "LastShot/TerminalX", 0.0);
      SmartDashboard.putNumber(kRoot + "LastShot/TerminalY", 0.0);
      SmartDashboard.putNumber(kRoot + "LastShot/TerminalZ", 0.0);
    }

    if (lastSweep.isPresent()) {
      ShotSweepResult sweep = lastSweep.get();
      SmartDashboard.putString(kRoot + "Sweep/Summary", sweep.summary());
      SmartDashboard.putString(kRoot + "Sweep/ResultsCsv", sweep.toCsv());
      if (sweep.bestHit().isPresent()) {
        ShotSampleResult best = sweep.bestHit().get();
        SmartDashboard.putNumber(kRoot + "Sweep/BestHitRPM", best.rpm());
        SmartDashboard.putNumber(kRoot + "Sweep/BestHitHoodDeg", best.hoodDeg());
        SmartDashboard.putNumber(kRoot + "Sweep/HitRate", sweep.hitRate());
      } else {
        SmartDashboard.putNumber(kRoot + "Sweep/BestHitRPM", 0.0);
        SmartDashboard.putNumber(kRoot + "Sweep/BestHitHoodDeg", 0.0);
        SmartDashboard.putNumber(kRoot + "Sweep/HitRate", 0.0);
      }
    } else {
      SmartDashboard.putString(kRoot + "Sweep/Summary", "No sweep run yet");
      SmartDashboard.putString(kRoot + "Sweep/ResultsCsv", "");
      SmartDashboard.putNumber(kRoot + "Sweep/BestHitRPM", 0.0);
      SmartDashboard.putNumber(kRoot + "Sweep/BestHitHoodDeg", 0.0);
      SmartDashboard.putNumber(kRoot + "Sweep/HitRate", 0.0);
    }
  }

  private static boolean consumeBoolean(String key) {
    boolean current = SmartDashboard.getBoolean(key, false);
    if (current) {
      SmartDashboard.putBoolean(key, false);
    }
    return current;
  }

  public record Inputs(
      Pose2d requestedPose,
      ChassisSpeeds requestedFieldSpeeds,
      double shooterRpm,
      double hoodDeg,
      double turretDeg,
      double launchHeightM,
      double shooterOffsetXM,
      double shooterOffsetYM,
      double launchSpeedAt6000RpmMps,
      boolean useIntakeAmmo,
      boolean intakeRunning,
      boolean applyPoseNow,
      boolean loadOneFuel,
      boolean fireNow,
      boolean resetField,
      boolean clearProjectiles,
      boolean runSweep,
      double sweepRpmMin,
      double sweepRpmMax,
      double sweepRpmStep,
      double sweepHoodMinDeg,
      double sweepHoodMaxDeg,
      double sweepHoodStepDeg) {}
}
