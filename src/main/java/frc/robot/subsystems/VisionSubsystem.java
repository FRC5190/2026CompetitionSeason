package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  private static final String kLimelightName = "limelight"; // change if you renamed in LL UI

  private double horizontalOffsetDegrees = 0.0;
  private double verticalOffsetDegrees = 0.0;
  private int trackedTagId = -1;
  private boolean targetVisible = false;

  @Override
  public void periodic() {
    // If your helper returns boolean tv, use it; otherwise use tagId == -1 as your validity
    targetVisible = LimelightHelpers.getTV(kLimelightName);

    // AprilTag ID (Limelight returns -1 when none, in your test)
    trackedTagId = (int) LimelightHelpers.getFiducialID(kLimelightName);

    // Horizontal offset (degrees)
    horizontalOffsetDegrees = LimelightHelpers.getTX(kLimelightName);
    verticalOffsetDegrees = LimelightHelpers.getTY(kLimelightName);

    // If you want: force validity from tagId instead
    if (trackedTagId < 0) {
      targetVisible = false;
    }

    // System.out.println("LL hasTarget=" + hasTarget + " id= " + closestTagId + " horizontalOffsetDegrees = " + horizontalOffsetDegrees);
  }

  public double getHorizontalOffsetDegrees() {
    return horizontalOffsetDegrees;
  }

  public double getVerticalOffsetDegrees() {
    return verticalOffsetDegrees;
  }

  public int getTrackedTagId() {
    return trackedTagId;
  }

  public boolean hasTarget() {
    return targetVisible;
  }

  public double getTargetDistanceMeters() {
    if (!targetVisible) {
      return -1.0;
    }

    double[] targetPose = LimelightHelpers.getBotPose_TargetSpace(kLimelightName);
    if (targetPose == null || targetPose.length < 3) {
      return -1.0;
    }

    return Math.hypot(targetPose[0], targetPose[2]);
  }
}
