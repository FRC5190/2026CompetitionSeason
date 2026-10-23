package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer extends SubsystemBase {

  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;

  private final PeriodicIO periodicIO = new PeriodicIO();

  public Indexer() {

    // --- Indexer Leader ---
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.voltageCompensation(12);
    leaderConfig.smartCurrentLimit(Constants.kCurrentLimit);
    leaderConfig.inverted(false);
    leaderConfig.idleMode(IdleMode.kCoast);

    leaderMotor = new SparkMax(Constants.kIndexerLeaderId, kBrushless);
    leaderEncoder = leaderMotor.getEncoder();
    leaderMotor.configure(leaderConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // --- Indexer Follower (same direction as leader) ---
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.voltageCompensation(12);
    followerConfig.smartCurrentLimit(Constants.kCurrentLimit);
    followerConfig.idleMode(IdleMode.kCoast);
    // followerConfig.follow(Constants.kIndexerLeaderId, false); // false = same
    // direction as
    // leader

    followerMotor = new SparkMax(Constants.kIndexerFollowerId, kBrushless);
    followerEncoder = followerMotor.getEncoder();
    followerMotor.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Read inputs
    periodicIO.leaderCurrentAmps = leaderMotor.getOutputCurrent();
    periodicIO.followerCurrentAmps = followerMotor.getOutputCurrent();
    periodicIO.leaderVelocityRpm = leaderEncoder.getVelocity();
    periodicIO.followerVelocityRpm = followerEncoder.getVelocity();

    // Write outputs — follower mirrors leader automatically
    leaderMotor.set(periodicIO.demand);
    followerMotor.set(periodicIO.demand);
  }

  /** Spin the indexer at a given percent output [-1, 1] */
  public void setPercent(double requestedPercent) {
    periodicIO.demand = requestedPercent;
  }

  /** Stop the indexer */
  public void stop() {
    periodicIO.demand = 0;
  }

  public double getLeaderCurrent() {
    return periodicIO.leaderCurrentAmps;
  }

  public double getFollowerCurrent() {
    return periodicIO.followerCurrentAmps;
  }

  public double getIndexerPercent() {
    return periodicIO.demand;
  }

  public double getLeaderVelocityRpm() {
    return periodicIO.leaderVelocityRpm;
  }

  public double getFollowerVelocityRpm() {
    return periodicIO.followerVelocityRpm;
  }

  public double getReverseCheckTimeRemainingSeconds() {
    return periodicIO.reverseCheckTimeRemainingSeconds;
  }

  public void resetReverseCheckTimer() {
    periodicIO.reverseCheckTimeRemainingSeconds = PeriodicIO.kReverseCheckIntervalSeconds;
  }

  public void decrementReverseCheckTimer(double elapsedSeconds) {
    periodicIO.reverseCheckTimeRemainingSeconds -= elapsedSeconds;
  }

  public void updateJamRecovery(double requestedPercent, double elapsedSeconds) {
    this.decrementReverseCheckTimer(elapsedSeconds);
    if (this.getReverseCheckTimeRemainingSeconds() < 0) {
      if (Math.abs(this.getLeaderVelocityRpm()) < 5) {
        this.setPercent(-requestedPercent);
      } else {
        this.setPercent(requestedPercent);
      }
      this.resetReverseCheckTimer();
    }
  }

  public static class PeriodicIO {
    // Inputs
    double leaderCurrentAmps;
    double followerCurrentAmps;
    double leaderVelocityRpm;
    double followerVelocityRpm;

    // Outputs
    double demand;

    // how much to wait before revserse if stuck
    public static final double kReverseCheckIntervalSeconds = 0.5;
    double reverseCheckTimeRemainingSeconds = kReverseCheckIntervalSeconds;
  }

  public static class Constants {
    public static final int kIndexerLeaderId = 11;
    public static final int kIndexerFollowerId = 12;
    public static final int kCurrentLimit = 30; // Keep between [20, 40]
  }
}
