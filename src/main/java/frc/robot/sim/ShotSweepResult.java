package frc.robot.sim;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public record ShotSweepResult(ShotSweepConfig config, List<ShotSampleResult> samples) {

  public int totalShots() {
    return samples.size();
  }

  public long hitCount() {
    return samples.stream().filter(ShotSampleResult::scored).count();
  }

  public double hitRate() {
    return samples.isEmpty() ? 0.0 : (double) hitCount() / samples.size();
  }

  public Optional<ShotSampleResult> bestHit() {
    return samples.stream()
        .filter(ShotSampleResult::scored)
        .min(Comparator.comparingDouble(ShotSampleResult::rpm)
            .thenComparingDouble(ShotSampleResult::hoodDeg));
  }

  public String toCsv() {
    StringBuilder builder =
        new StringBuilder("rpm,hoodDeg,turretDeg,launchSpeedMps,poseX,poseY,headingDeg,result,flightTimeSec,terminalX,terminalY,terminalZ");
    for (ShotSampleResult sample : samples) {
      builder.append('\n').append(sample.toCsvRow());
    }
    return builder.toString();
  }

  public String summary() {
    StringBuilder builder =
        new StringBuilder(String.format("shots=%d hits=%d hitRate=%.2f%%",
            totalShots(), hitCount(), hitRate() * 100.0));
    bestHit().ifPresent(best -> builder.append(
        String.format(" bestHit[rpm=%.1f hood=%.2f turret=%.2f]",
            best.rpm(), best.hoodDeg(), best.turretDeg())));
    return builder.toString();
  }
}
