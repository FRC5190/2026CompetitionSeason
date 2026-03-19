package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Turret turret_;
  private final SwerveSubsystem drivebase_;
  private final VisionSubsystem vision_;

  private final Pose2d goal_pose_;

  // Tuned constants
  private static final double kTotalExitVelocity = 15.0; // m/s - tune to your robot
  private static final double kLatency = 0.15; // seconds - tune to your robot

  // Physical Limelight mounting constants - MUST BE MEASURED ON YOUR ROBOT
  // private static final double kLimelightHeightInches = 24.0; // height of limelight lens from
  // floor
  // private static final double kTargetHeightInches = 60.0; // height of target center from floor
  // private static final double kLimelightAngleDegrees = 30.0; // limelight mounting angle from
  // horizontal

  private final InterpolatingDoubleTreeMap shooter_table_ = new InterpolatingDoubleTreeMap();

  public ShootOnTheMove(Turret turret, SwerveSubsystem drivebase, VisionSubsystem vision,
      Pose2d goalPose) {
    turret_ = turret;
    drivebase_ = drivebase;
    vision_ = vision;
    goal_pose_ = goalPose;

    // Distance (meters) -> Flywheel percent output
    // Tune these values on your actual robot!
    for (var entry : List.of(Pair.of(1.0, 0.4), Pair.of(2.0, 0.6), Pair.of(3.0, 0.8))) {
      shooter_table_.put(entry.getFirst(), entry.getSecond());
    }

    addRequirements(turret_);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Get current robot speed (field oriented)
    ChassisSpeeds robot_speed = drivebase_.getSwerveDrive().getFieldVelocity();

    // 1. LATENCY COMPENSATION
    Translation2d future_pos = drivebase_.getSwerveDrive().getPose().getTranslation()
        .plus(new Translation2d(robot_speed.vxMetersPerSecond, robot_speed.vyMetersPerSecond)
            .times(kLatency));

    // 2. GET DISTANCE
    // If Limelight sees the target, use ty for a direct distance measurement
    // Otherwise fall back to odometry based distance
    double dist;
    if (vision_.hasTag()) {
      // dist = calculateDistanceFromTY(vision_.getTY());
      double[] target_pose = LimelightHelpers.getBotPose_TargetSpace("limelight");
      dist = Math.sqrt(Math.pow(target_pose[0], 2) + Math.pow(target_pose[2], 2));
    } else {
      Translation2d goal_location = goal_pose_.getTranslation();
      Translation2d target_vec = goal_location.minus(future_pos);
      dist = target_vec.getNorm();
    }

    // 3. LOOK UP IDEAL FLYWHEEL PERCENT FOR THIS DISTANCE
    double ideal_flywheel_percent = shooter_table_.get(dist);

    // 4. VECTOR SUBTRACTION
    // Use Limelight tx directly for turret angle when tag is visible
    // Otherwise calculate from odometry
    double turret_angle;
    double new_horizontal_speed;

    if (vision_.hasTag()) {
      // Limelight tx is the horizontal angle offset to the target
      // Subtract robot velocity compensation on top of direct tx
      Translation2d robot_vel_vec =
          new Translation2d(robot_speed.vxMetersPerSecond, robot_speed.vyMetersPerSecond);

      // Convert tx to a unit vector and scale by ideal speed, then subtract robot velocity
      double tx_radians = Math.toRadians(vision_.getTX())
          + Math.toRadians(turret_.getRotationPosition());
      Translation2d shot_vec =
          new Translation2d(Math.cos(tx_radians) * ideal_flywheel_percent * kTotalExitVelocity,
              Math.sin(tx_radians) * ideal_flywheel_percent * kTotalExitVelocity)
                  .minus(robot_vel_vec);

      turret_angle = shot_vec.getAngle().getDegrees();
      new_horizontal_speed = shot_vec.getNorm();
    } else {
      // Fall back to odometry based vector math
      Translation2d goal_location = goal_pose_.getTranslation();
      Translation2d target_vec = goal_location.minus(future_pos);
      Translation2d robot_vel_vec =
          new Translation2d(robot_speed.vxMetersPerSecond, robot_speed.vyMetersPerSecond);
      Translation2d shot_vec = target_vec.div(dist)
          .times(ideal_flywheel_percent * kTotalExitVelocity).minus(robot_vel_vec);

      turret_angle = shot_vec.getAngle().getDegrees();
      new_horizontal_speed = shot_vec.getNorm();
    }

    // 5. SOLVE FOR HOOD ANGLE
    double ratio = Math.min(new_horizontal_speed / kTotalExitVelocity, 1.0);
    double new_pitch = Math.toDegrees(Math.acos(ratio));

    // 6. SET OUTPUTS
    turret_.setRotationPosition(turret_angle);
    turret_.setHoodPosition(new_pitch);
    turret_.setFlywheelPercent(ideal_flywheel_percent);

    // --- AdvantageScope logging ---
    SmartDashboard.putNumberArray("Turret/FieldPose", new double[] {
        drivebase_.getSwerveDrive().getPose().getX(),
        drivebase_.getSwerveDrive().getPose().getY(),
        Math.toRadians(turret_angle)
    });
    SmartDashboard.putNumberArray("Hub/FieldPose", new double[] {
        goal_pose_.getX(),
        goal_pose_.getY(),
        0.0
    });
    SmartDashboard.putNumber("SOTM/TurretAngle", turret_angle);
    SmartDashboard.putNumber("SOTM/Distance", dist);
    SmartDashboard.putNumber("SOTM/HoodAngle", new_pitch);
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
    turret_.stopAll();
  }
}
