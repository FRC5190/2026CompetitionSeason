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

  private final SparkMax indexer_leader_;
  private final SparkMax indexer_follower_;
  private final RelativeEncoder leader_encoder_;
  private final RelativeEncoder follower_encoder_;

  private final PeriodicIO io_ = new PeriodicIO();

  public Indexer() {

    // --- Indexer Leader ---
    SparkMaxConfig leader_config = new SparkMaxConfig();
    leader_config.voltageCompensation(12);
    leader_config.smartCurrentLimit(Constants.kCurrentLimit);
    leader_config.inverted(false);
    leader_config.idleMode(IdleMode.kCoast);

    indexer_leader_ = new SparkMax(Constants.kIndexerLeaderId, kBrushless);
    leader_encoder_ = indexer_leader_.getEncoder();
    indexer_leader_.configure(leader_config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // --- Indexer Follower (same direction as leader) ---
    SparkMaxConfig follower_config = new SparkMaxConfig();
    follower_config.voltageCompensation(12);
    follower_config.smartCurrentLimit(Constants.kCurrentLimit);
    follower_config.idleMode(IdleMode.kCoast);
    // follower_config.follow(Constants.kIndexerLeaderId, false); // false = same direction as
    // leader

    indexer_follower_ = new SparkMax(Constants.kIndexerFollowerId, kBrushless);
    follower_encoder_ = indexer_follower_.getEncoder();
    indexer_follower_.configure(follower_config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.current_leader_ = indexer_leader_.getOutputCurrent();
    io_.current_follower_ = indexer_follower_.getOutputCurrent();
    io_.leader_velocity_rpm_ = leader_encoder_.getVelocity();
    io_.follower_velocity_rpm_ = follower_encoder_.getVelocity();

    // Write outputs — follower mirrors leader automatically
    indexer_leader_.set(io_.indexer_demand_);
    indexer_follower_.set(io_.indexer_demand_);
  }

  /** Spin the indexer at a given percent output [-1, 1] */
  public void setPercent(double value) {
    io_.indexer_demand_ = value;
  }

  /** Stop the indexer */
  public void stop() {
    io_.indexer_demand_ = 0;
  }

  public double getLeaderCurrent() {
    return io_.current_leader_;
  }

  public double getFollowerCurrent() {
    return io_.current_follower_;
  }

  public double getIndexerPercent() {
    return io_.indexer_demand_;
  }

  public double getLeaderVelocityRpm() {
    return io_.leader_velocity_rpm_;
  }

  public double getFollowerVelocityRpm() {
    return io_.follower_velocity_rpm_;
  }

  

  public static class PeriodicIO {
    // Inputs
    double current_leader_;
    double current_follower_;
    double leader_velocity_rpm_;
    double follower_velocity_rpm_;

    // Outputs
    double indexer_demand_;
  }

  public static class Constants {
    public static final int kIndexerLeaderId = 11;
    public static final int kIndexerFollowerId = 12;
    public static final int kCurrentLimit = 30; // Keep between [20, 40]
  }
}
