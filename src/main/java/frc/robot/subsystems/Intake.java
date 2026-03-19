package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {

  // Roller motor 
  private final SparkMax roller_;

  // Extension motor
  private final SparkMax extension_;

  private final PeriodicIO io_ = new PeriodicIO();

  public Intake() {

    // Roller
    SparkMaxConfig roller_config = new SparkMaxConfig();
    roller_config.voltageCompensation(12);
    roller_config.smartCurrentLimit(20);
    roller_config.inverted(false);
    roller_config.idleMode(IdleMode.kBrake);

    roller_ = new SparkMax(Constants.kRollerId, kBrushless);
    roller_.configure(roller_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Extension 
    SparkMaxConfig extension_config = new SparkMaxConfig();
    extension_config.voltageCompensation(12);
    extension_config.smartCurrentLimit(20);
    extension_config.inverted(false);
    extension_config.idleMode(IdleMode.kBrake);

    extension_ = new SparkMax(Constants.kExtensionId, kBrushless);
    extension_.configure(extension_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.current_roller_    = roller_.getOutputCurrent();
    io_.current_extension_ = extension_.getOutputCurrent();

    // Write outputs
    roller_.set(io_.roller_demand_);
    extension_.set(io_.extension_demand_);
  }

  /** Spin the roller at a given percent output [-1, 1] */
  public void setRollerPercent(double value) {
    io_.roller_demand_ = value;
  }

  /** Drive the extension motor at a given percent output [-1, 1] */
  public void setExtensionPercent(double value) {
    io_.extension_demand_ = value;
  }

  /** Stop all motors */
  public void stopAll() {
    io_.roller_demand_    = 0;
    io_.extension_demand_ = 0;
  }

  /** Stop only the roller */
  public void stopRoller() {
    io_.roller_demand_ = 0;
  }

  /** Stop only the extension motor */
  public void stopExtension() {
    io_.extension_demand_ = 0;
  }

  public double getRollerCurrent()    { return io_.current_roller_;    }
  public double getExtensionCurrent() { return io_.current_extension_; }

  public static class PeriodicIO {
    // Inputs
    double current_roller_;
    double current_extension_;

    // Outputs
    double roller_demand_;
    double extension_demand_;
  }

  public static class Constants {
    public static final int kRollerId    = 9;
    public static final int kExtensionId = 10;
  }
}