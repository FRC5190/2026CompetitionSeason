package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;

/**
 * Shoot on the move command. Largely based off
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly Uses Limelight tx directly for turret
 * angle and ty for distance calculation.
 */
public class ShootOnTheMove extends Command {

  private final Turret turret;
  private final SwerveSubsystem drivebase;
  private final VisionSubsystem vision;

  private final Pose2d goalPose;

  // Tuned constants
  private static final double kTotalExitVelocityMetersPerSecond = 15.0; // m/s - tune to your robot
  private static final double kLatencySeconds = 0.15; // seconds - tune to your robot

  // Physical Limelight mounting constants - MUST BE MEASURED ON YOUR ROBOT
  // private static final double kLimelightHeightInches = 24.0; // height of limelight lens from
  // floor
  // private static final double kTargetHeightInches = 60.0; // height of target center from floor
  // private static final double kLimelightAngleDegrees = 30.0; // limelight mounting angle from
  // horizontal

  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  public ShootOnTheMove(Turret turret, SwerveSubsystem drivebase, VisionSubsystem vision,
      Pose2d goalPose) {
    this.turret = turret;
    this.drivebase = drivebase;
    this.vision = vision;
    this.goalPose = goalPose;

    // Distance (meters) -> Flywheel percent output
    // Tune these values on your actual robot!
    for (var entry : List.of(Pair.of(1.0, 0.4), Pair.of(2.0, 0.6), Pair.of(3.0, 0.8))) {
      shooterTable.put(entry.getFirst(), entry.getSecond());
    }

    addRequirements(this.turret, this.drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Get current robot speed (field oriented)
    ChassisSpeeds fieldVelocity = drivebase.getSwerveDrive().getFieldVelocity();

    // 1. LATENCY COMPENSATION
    Translation2d projectedRobotPosition = drivebase.getSwerveDrive().getPose().getTranslation()
        .plus(new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond)
            .times(kLatencySeconds));

    // 2. GET DISTANCE
    // If Limelight sees the target, use ty for a direct distance measurement
    // Otherwise fall back to odometry based distance
    double targetDistanceMeters;
    if (vision.hasTarget()) {
      // targetDistanceMeters = calculateDistanceFromTY(vision.getTY());
      double[] targetSpacePose = LimelightHelpers.getBotPose_TargetSpace("limelight");
      targetDistanceMeters = Math.sqrt(Math.pow(targetSpacePose[0], 2) + Math.pow(targetSpacePose[2], 2));
    } else {
      Translation2d goalLocation = goalPose.getTranslation();
      Translation2d targetVector = goalLocation.minus(projectedRobotPosition);
      targetDistanceMeters = targetVector.getNorm();
    }

    // 3. LOOK UP IDEAL FLYWHEEL PERCENT FOR THIS DISTANCE
    double idealFlywheelPercent = shooterTable.get(targetDistanceMeters);

    // 4. VECTOR SUBTRACTION
    // Use Limelight tx directly for turret angle when tag is visible
    // Otherwise calculate from odometry
    double turretAngleDegrees;
    double shotSpeedMetersPerSecond;

    if (vision.hasTarget()) {
      // Limelight tx is the horizontal angle offset to the target
      // Subtract robot velocity compensation on top of direct tx
      Translation2d robotVelocity =
          new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

      // Convert tx to a unit vector and scale by ideal speed, then subtract robot velocity
      double targetBearingOffsetRadians = Math.toRadians(vision.getHorizontalOffsetDegrees());
      Translation2d shotVector =
          new Translation2d(Math.cos(targetBearingOffsetRadians) * idealFlywheelPercent * kTotalExitVelocityMetersPerSecond,
              Math.sin(targetBearingOffsetRadians) * idealFlywheelPercent * kTotalExitVelocityMetersPerSecond)
                  .minus(robotVelocity);

      turretAngleDegrees = shotVector.getAngle().getDegrees();
      shotSpeedMetersPerSecond = shotVector.getNorm();
    } else {
      // Fall back to odometry based vector math
      Translation2d goalLocation = goalPose.getTranslation();
      Translation2d targetVector = goalLocation.minus(projectedRobotPosition);
      Translation2d robotVelocity =
          new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
      Translation2d shotVector = targetVector.div(targetDistanceMeters)
          .times(idealFlywheelPercent * kTotalExitVelocityMetersPerSecond).minus(robotVelocity);

      turretAngleDegrees = shotVector.getAngle().getDegrees();
      shotSpeedMetersPerSecond = shotVector.getNorm();
    }

    // 5. SOLVE FOR HOOD ANGLE
    double velocityRatio = Math.min(shotSpeedMetersPerSecond / kTotalExitVelocityMetersPerSecond, 1.0);
    double hoodPitchDegrees = Math.toDegrees(Math.acos(velocityRatio));

    // 6. SET OUTPUTS
    turret.setRotationPosition(turretAngleDegrees);
    turret.setHoodPosition(hoodPitchDegrees);
    turret.setFlywheelPercent(idealFlywheelPercent);
  }

  /**
   * Calculate distance to target in meters using Limelight ty Based on: distance = (targetHeight -
   * limelightHeight) / tan(limelightAngle + ty)
   */
  // private double calculateDistanceFromTY(double ty) {
  // double angle_to_target_degrees = kLimelightAngleDegrees + ty;
  // double distance_inches = (kTargetHeightInches - kLimelightHeightInches) /
  // Math.tan(Math.toRadians(angle_to_target_degrees));
  // return distance_inches * 0.0254; // convert inches to meters
  // }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopAll();
  }
}
