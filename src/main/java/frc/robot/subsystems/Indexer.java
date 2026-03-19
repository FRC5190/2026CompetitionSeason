package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer extends SubsystemBase {

  private final SparkMax indexer_;

  private final PeriodicIO io_ = new PeriodicIO();

  public Indexer() {

    // Indexer
    SparkMaxConfig indexer_config = new SparkMaxConfig();
    indexer_config.voltageCompensation(12);
    indexer_config.smartCurrentLimit(20);
    indexer_config.inverted(false);
    indexer_config.idleMode(IdleMode.kBrake);

    indexer_ = new SparkMax(Constants.kIndexerId, kBrushless);
    indexer_.configure(indexer_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.current_indexer_ = indexer_.getOutputCurrent();

    // Write outputs
    indexer_.set(io_.indexer_demand_);
  }

  /** Spin the indexer at a given percent output [-1, 1] */
  public void setPercent(double value) {
    io_.indexer_demand_ = value;
  }

  /** Stop the indexer */
  public void stop() {
    io_.indexer_demand_ = 0;
  }

  public double getIndexerCurrent() {
    return io_.current_indexer_;
  }

  public static class PeriodicIO {
    // Inputs
    double current_indexer_;

    // Outputs
    double indexer_demand_;
  }

  public static class Constants {
    public static final int kIndexerId = 11;
  }
}
