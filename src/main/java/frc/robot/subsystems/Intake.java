package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {

  // Intake Motors
  private final SparkMax roller_motor_;
  private final SparkMax extension_motor_;

  // Intake Encoder
  private final RelativeEncoder extension_encoder_;
  private final RelativeEncoder roller_encoder_;

  private final PIDController extension_pid_;

  // Periodic IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  public Intake() {
    // Roller
    SparkMaxConfig roller_config = new SparkMaxConfig();
    roller_config.voltageCompensation(12);
    roller_config.inverted(false);
    roller_config.idleMode(IdleMode.kBrake);

    roller_motor_ = new SparkMax(Constants.kRollerId, kBrushless);
    roller_encoder_ = roller_motor_.getEncoder();


    // Extension
    SparkMaxConfig extension_config = new SparkMaxConfig();
    extension_config.voltageCompensation(12);
    extension_config.inverted(false);
    extension_config.idleMode(IdleMode.kBrake);
    extension_config.encoder.positionConversionFactor(1.0 / Constants.kExtensionGearRatio);
    extension_config.encoder.velocityConversionFactor(1.0 / Constants.kExtensionGearRatio);

    extension_motor_ = new SparkMax(Constants.kExtensionId, kBrushless);
    extension_encoder_ = extension_motor_.getEncoder();

    // Safety
    roller_config.smartCurrentLimit(Constants.kRollerCurrentLimit);
    extension_config.smartCurrentLimit(Constants.kExtensionCurrentLimit);
    extension_config.softLimit.forwardSoftLimit(Constants.kMaxExtensionPosition); // Double check this
    extension_config.softLimit.reverseSoftLimit(Constants.kMinExtensionPosition); // Double check this

    // Configuration

    extension_motor_.configure(extension_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    roller_motor_.configure(roller_config, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    extension_pid_ = new PIDController(Constants.kExtensionP, 0.0, Constants.kExtensionD);
    extension_pid_.setTolerance(Constants.kExtensionTolerance);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.current_roller_ = roller_motor_.getOutputCurrent();
    io_.current_extension_ = extension_motor_.getOutputCurrent();

    io_.current_extension_position_ = extension_encoder_.getPosition();
    io_.current_roller_velocity_ = roller_encoder_.getVelocity();

    // Roller Outputs
    roller_motor_.set(io_.roller_demand_);

    // Extension Outputs
    switch (output_type_) {
      case DISTANCE:
        extension_pid_.setSetpoint(io_.extension_target_);
        double output = MathUtil.clamp(
            extension_pid_.calculate(io_.current_extension_position_, io_.extension_target_), -0.2,
            0.2);
        io_.extension_demand_ = output;

        extension_motor_.set(io_.extension_demand_);
        break;
      case BRAKE:
        io_.extension_demand_ = 0;

        extension_motor_.set(io_.extension_demand_);
        break;
      case PERCENT:
        extension_motor_.set(io_.extension_demand_);
        break;
    }
  }

  /** Spin the roller at a given percent output [-1, 1] */
  public void setRollerPercent(double value) {
    io_.roller_demand_ = value;
  }

  /** Drive the extension motor at a given percent output [-1, 1] */
  public void setExtensionPercent(double value) {
    output_type_ = OutputType.PERCENT;
    io_.extension_demand_ = value;
  }

  /** Move the extension to a target position in output-shaft rotations. */
  public void setExtensionPosition(double position) {
    output_type_ = OutputType.DISTANCE;
    io_.extension_target_ =
        MathUtil.clamp(position, Constants.kMinExtensionPosition, Constants.kMaxExtensionPosition);
  }

  /** Hold the extension at its current position. */
  public void setExtensionBrake() {
    setExtensionBrake(io_.current_extension_position_);
  }

  /** Hold the extension at a specific position in output-shaft rotations. */
  public void setExtensionBrake(double position) {
    output_type_ = OutputType.BRAKE;
  }

  /** Stop all motors */
  public void stopAll() {
    io_.roller_demand_ = 0;
    io_.extension_demand_ = 0;
  }

  /** Stop only the roller */
  public void stopRoller() {
    io_.roller_demand_ = 0;
  }

  /** Stop only the extension motor */
  public void stopExtension() {
    output_type_ = OutputType.PERCENT;
    io_.extension_demand_ = 0;
  }

  public double getRollerCurrent() {
    return io_.current_roller_;
  }

  public double getExtensionCurrent() {
    return io_.current_extension_;
  }

  public double getExtensionPosition() {
    return io_.current_extension_position_;
  }

  public double getRollerVelocity() {
    return io_.current_roller_velocity_;
  }

  public double getRollerDemand() {
    return io_.roller_demand_;
  }

  public double getExtensionDemand() {
    return io_.extension_demand_;
  }

  public double getExtensionTarget() {
    return io_.extension_target_;
  }

  public boolean isExtensionAtTarget() {
    return output_type_ != OutputType.PERCENT && extension_pid_.atSetpoint();
  }

  private enum OutputType {
    PERCENT, DISTANCE, BRAKE
  }

  public static class PeriodicIO {
    // Inputs
    double current_roller_;
    double current_extension_;
    double current_extension_position_;
    double current_roller_velocity_;

    // Outputs
    double roller_demand_;
    double extension_demand_;
    double extension_target_;
  }

  public static class Constants {
    // Extension Constants
    public static final int kExtensionId = 10;
    public static final double kExtensionGearRatio = 5.0; // CHANGE
    public static final double kMaxExtensionPosition = 10.0; // CHANGE
    public static final double kMinExtensionPosition = -0.2; // CHANGE

    public static final double kExtensionP = 0.5;
    public static final double kExtensionD = 0.0;
    public static final double kExtensionTolerance = 0.1;
    public static final double kMaxExtensionOutput = 0.4;
    public static final double kMaxBrakeOutput = 0.2;

    public static final int kExtensionCurrentLimit = 30; // Keep between [<20, 40] MAX 40

    // Roller Constants
    public static final int kRollerId = 9;

    public static final int kRollerCurrentLimit = 30; // Keep between [<20, 40] MAX 40
  }
}
