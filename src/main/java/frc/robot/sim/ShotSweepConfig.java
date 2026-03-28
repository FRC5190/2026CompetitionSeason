package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;

public record ShotSweepConfig(
    double rpmMin,
    double rpmMax,
    double rpmStep,
    double hoodMinDeg,
    double hoodMaxDeg,
    double hoodStepDeg,
    double turretDeg,
    double launchHeightM,
    double shooterOffsetXM,
    double shooterOffsetYM,
    double launchSpeedAt6000RpmMps,
    Pose2d robotPose,
    ChassisSpeeds chassisSpeedsFieldRelative) {

  public ShotSweepConfig normalize() {
    double normalizedRpmMin = Math.min(rpmMin, rpmMax);
    double normalizedRpmMax = Math.max(rpmMin, rpmMax);
    double normalizedHoodMin = Math.min(hoodMinDeg, hoodMaxDeg);
    double normalizedHoodMax = Math.max(hoodMinDeg, hoodMaxDeg);

    return new ShotSweepConfig(
        normalizedRpmMin,
        normalizedRpmMax,
        Math.max(1.0, Math.abs(rpmStep)),
        normalizedHoodMin,
        normalizedHoodMax,
        Math.max(0.1, Math.abs(hoodStepDeg)),
        turretDeg,
        launchHeightM,
        shooterOffsetXM,
        shooterOffsetYM,
        launchSpeedAt6000RpmMps,
        robotPose,
        chassisSpeedsFieldRelative);
  }

  public List<Double> rpmValues() {
    return valuesInRange(rpmMin, rpmMax, rpmStep, 0.25);
  }

  public List<Double> hoodValuesDeg() {
    return valuesInRange(hoodMinDeg, hoodMaxDeg, hoodStepDeg, 1e-3);
  }

  private static List<Double> valuesInRange(
      double minValue, double maxValue, double stepValue, double epsilon) {
    List<Double> values = new ArrayList<>();
    for (double value = minValue; value <= maxValue + epsilon; value += stepValue) {
      values.add(Math.min(value, maxValue));
    }
    if (values.isEmpty()) {
      values.add(minValue);
    }
    return values;
  }
}
