package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record ShotSampleResult(
    double rpm,
    double hoodDeg,
    double turretDeg,
    double launchSpeedMps,
    Pose2d robotPose,
    ChassisSpeeds chassisSpeedsFieldRelative,
    boolean scored,
    double flightTimeSec,
    Pose3d terminalPose) {

  public String toCsvRow() {
    return String.format(
        "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%.3f,%.3f",
        rpm,
        hoodDeg,
        turretDeg,
        launchSpeedMps,
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation().getDegrees(),
        scored ? "hit" : "miss",
        flightTimeSec,
        terminalPose.getX(),
        terminalPose.getY(),
        terminalPose.getZ());
  }
}
