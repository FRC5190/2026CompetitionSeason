package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  private static final String kLL = "limelight"; // change if you renamed in LL UI

  private double tx = 0.0;
  private double ty = 0.0;
  private int closestTagID = -1;
  private boolean hasTag = false;

  @Override
  public void periodic() {
    // If your helper returns boolean tv, use it; otherwise use tagId == -1 as your validity
    hasTag = LimelightHelpers.getTV(kLL);

    // AprilTag ID (Limelight returns -1 when none, in your test)
    closestTagID = (int) LimelightHelpers.getFiducialID(kLL);

    // Horizontal offset (degrees)
    tx = LimelightHelpers.getTX(kLL);
    ty = LimelightHelpers.getTY(kLL);

    // If you want: force validity from tagId instead
    if (closestTagID < 0) {
      hasTag = false;
    }

    // System.out.println("LL hasTag=" + hasTag + " id= " + closestTagID + " tx = " + tx);
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public int getClosestTagID() {
    return closestTagID;
  }

  public boolean hasTag() {
    return hasTag;
  }

  public double getTargetDistanceMeters() {
    if (!hasTag) {
      return -1.0;
    }

    double[] targetPose = LimelightHelpers.getBotPose_TargetSpace(kLL);
    if (targetPose == null || targetPose.length < 3) {
      return -1.0;
    }

    return Math.hypot(targetPose[0], targetPose[2]);
  }
}
