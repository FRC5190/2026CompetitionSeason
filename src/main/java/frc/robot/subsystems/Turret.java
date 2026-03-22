package frc.robot.subsystems;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  // Flywheel motors
  private final SparkMax flywheel_leader_;
  private final SparkMax flywheel_follower_;
  private final RelativeEncoder flywheel_leader_encoder_;
  private final RelativeEncoder flywheel_follower_encoder_;

  // Hood motor
  private final SparkMax hood_;
  private final RelativeEncoder hood_encoder_;
  private final PIDController hood_pid_;

  // Rotation motor
  private final SparkMax rotation_;
  private final RelativeEncoder rotation_encoder_;
  private final PIDController rotation_pid_;
  private final SparkAbsoluteEncoder bore_encoder_;

  // Periodic IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType hood_output_type_ = OutputType.PERCENT;
  private OutputType rotation_output_type_ = OutputType.BRAKE;

  public Turret() {
    // Flywheel leader
    SparkMaxConfig flywheel_leader_config = new SparkMaxConfig();
    flywheel_leader_config.voltageCompensation(Constants.kVoltageCompensation);
    flywheel_leader_config.smartCurrentLimit(Constants.kFlywheelCurrentLimit);
    flywheel_leader_config.inverted(Constants.kFlywheelLeaderInverted);
    flywheel_leader_config.idleMode(IdleMode.kCoast);
    flywheel_leader_config.encoder.positionConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheel_leader_config.encoder.velocityConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheel_leader_ = new SparkMax(Constants.kFlywheelLeaderId, kBrushless);
    flywheel_leader_encoder_ = flywheel_leader_.getEncoder();

    flywheel_leader_.configure(flywheel_leader_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Flywheel follower
    SparkMaxConfig flywheel_follower_config = new SparkMaxConfig();
    flywheel_follower_config.voltageCompensation(Constants.kVoltageCompensation);
    flywheel_follower_config.smartCurrentLimit(Constants.kFlywheelCurrentLimit);
    flywheel_follower_config.idleMode(IdleMode.kCoast);
    flywheel_follower_config.encoder.positionConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheel_follower_config.encoder.velocityConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheel_follower_config.follow(Constants.kFlywheelLeaderId,
        Constants.kFlywheelFollowerOpposesLeader);

    flywheel_follower_ = new SparkMax(Constants.kFlywheelFollowerId, kBrushless);
    flywheel_follower_encoder_ = flywheel_follower_.getEncoder();
    flywheel_follower_.configure(flywheel_follower_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Hood
    SparkMaxConfig hood_config = new SparkMaxConfig();
    hood_config.voltageCompensation(Constants.kVoltageCompensation);
    hood_config.smartCurrentLimit(Constants.kHoodCurrentLimit);
    hood_config.inverted(Constants.kHoodInverted);
    hood_config.idleMode(IdleMode.kBrake);
    hood_config.encoder.positionConversionFactor(360.0 / Constants.kHoodGearRatio);
    hood_config.encoder.velocityConversionFactor((360.0 / Constants.kHoodGearRatio) / 60.0);

    hood_ = new SparkMax(Constants.kHoodId, kBrushless);
    hood_.configure(hood_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hood_encoder_ = hood_.getEncoder();
    hood_encoder_.setPosition(Constants.kHoodStartingPosition);

    // Rotation
    SparkMaxConfig rotation_config = new SparkMaxConfig();
    rotation_config.voltageCompensation(Constants.kVoltageCompensation);
    rotation_config.smartCurrentLimit(Constants.kRotationCurrentLimit);
    rotation_config.inverted(Constants.kRotationInverted);
    rotation_config.idleMode(IdleMode.kBrake);
    rotation_config.encoder.positionConversionFactor(360.0 / Constants.kRotationGearRatio);
    rotation_config.encoder.velocityConversionFactor((360.0 / Constants.kRotationGearRatio) / 60.0);
    rotation_config.apply(
        new AbsoluteEncoderConfig().apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2)
            .positionConversionFactor(2.0 * Math.PI) // rotations -> radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0) // rpm -> rad/s
            .zeroOffset(Constants.kEncoderOffset));

    rotation_ = new SparkMax(Constants.kRotationId, kBrushless);
    rotation_.configure(rotation_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bore_encoder_ = rotation_.getAbsoluteEncoder();

    rotation_encoder_ = rotation_.getEncoder();
    rotation_encoder_.setPosition(Constants.kRotationStartingPosition);

    hood_pid_ = new PIDController(Constants.kHoodP, Constants.kHoodI, Constants.kHoodD);
    hood_pid_.setTolerance(Constants.kHoodTolerance);

    rotation_pid_ =
        new PIDController(Constants.kRotationP, Constants.kRotationI, Constants.kRotationD);
    rotation_pid_.setTolerance(Constants.kRotationTolerance);

    // Assign absolute encoder value to rotation encoder
    resetEncoder();
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.flywheel_leader_velocity_ = flywheel_leader_encoder_.getVelocity();
    io_.flywheel_follower_velocity_ = flywheel_follower_encoder_.getVelocity();
    io_.hood_position_ = hood_encoder_.getPosition();
    io_.rotation_position_ = rotation_encoder_.getPosition();

    io_.current_flywheel_leader_ = flywheel_leader_.getOutputCurrent();
    io_.current_flywheel_follower_ = flywheel_follower_.getOutputCurrent();
    io_.current_hood_ = hood_.getOutputCurrent();
    io_.current_rotation_ = rotation_.getOutputCurrent();

    // Flywheel outputs
    io_.flywheel_output_ = MathUtil.clamp(io_.flywheel_demand_, -Constants.kMaxFlywheelOutput,
        Constants.kMaxFlywheelOutput);

    // Hood outputs
    switch (hood_output_type_) {
      case POSITION:
        io_.hood_output_ = MathUtil.clamp(hood_pid_.calculate(io_.hood_position_, io_.hood_target_),
            -Constants.kMaxHoodOutput, Constants.kMaxHoodOutput);
        break;
      case PERCENT:
        io_.hood_output_ =
            MathUtil.clamp(io_.hood_demand_, -Constants.kMaxHoodOutput, Constants.kMaxHoodOutput);
        break;
      case BRAKE:
      default:
        hood_pid_.setSetpoint(getHoodPosition());
        io_.hood_output_ = MathUtil.clamp(hood_pid_.calculate(io_.hood_position_), -0.1, 0.1);
    }

    // Rotation outputs
    switch (rotation_output_type_) {
      case POSITION:
        io_.rotation_output_ =
            MathUtil.clamp(rotation_pid_.calculate(io_.rotation_position_, io_.rotation_target_),
                -Constants.kMaxRotationOutput, Constants.kMaxRotationOutput);
        break;
      case PERCENT:
        io_.rotation_output_ = MathUtil.clamp(io_.rotation_demand_, -Constants.kMaxRotationOutput,
            Constants.kMaxRotationOutput);
        break;
      case BRAKE:
      default:
        rotation_pid_.setSetpoint(getRotationPosition());
        io_.rotation_output_ = MathUtil.clamp(rotation_pid_.calculate(io_.rotation_position_), -0.1, 0.1);
    }

    // Hood soft limits
    if (io_.hood_position_ >= Constants.kMaxHoodPosition && io_.hood_output_ > 0) {
      io_.hood_output_ = 0;
    } else if (io_.hood_position_ <= Constants.kMinHoodPosition && io_.hood_output_ < 0) {
      io_.hood_output_ = 0;
    }

    // Rotation soft limits
    if (io_.rotation_position_ >= Constants.kMaxRotationPosition && io_.rotation_output_ > 0) {
      io_.rotation_output_ = 0;
    } else if (io_.rotation_position_ <= Constants.kMinRotationPosition
        && io_.rotation_output_ < 0) {
      io_.rotation_output_ = 0;
    }

    // Write outputs
    flywheel_leader_.set(io_.flywheel_output_);
    hood_.set(io_.hood_output_);
    rotation_.set(io_.rotation_output_);
  }

  /**
   * Returns the turret absolute encoder angle in radians, normalized to {@code [-pi, pi)}.
   *
   * @return Absolute turret angle in radians.
   */
  public double getEncoderRadians() {
    return bore_encoder_.getPosition();
  }

  /**
   * Synchronizes the relative turret rotation encoder to the absolute encoder reading.
   *
   * <p>
   * The relative encoder is stored in degrees, so the absolute encoder radians are converted before
   * writing.
   */
  public void resetEncoder() {
    rotation_encoder_.setPosition(Math.toDegrees(getEncoderRadians()));
  }

  // Flywheel
  /**
   * Sets the flywheel open-loop output.
   *
   * @param percent Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setFlywheelPercent(double percent) {
    io_.flywheel_demand_ = percent;
  }

  public void stopFlywheel() {
    io_.flywheel_demand_ = 0;
  }

  /**
   * Returns the currently applied flywheel output after clamping.
   *
   * @return Motor output in the range {@code [-1.0, 1.0]}.
   */
  public double getFlywheelPercent() {
    return io_.flywheel_output_;
  }

  /**
   * Returns the requested flywheel output before clamping.
   *
   * @return Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public double getFlywheelDemand() {
    return io_.flywheel_demand_;
  }

  /**
   * Returns the leader flywheel wheel speed.
   *
   * @return Flywheel velocity in wheel rotations per minute.
   */
  public double getFlywheelLeaderVelocity() {
    return io_.flywheel_leader_velocity_;
  }

  /**
   * Returns the follower flywheel wheel speed.
   *
   * @return Flywheel velocity in wheel rotations per minute.
   */
  public double getFlywheelFollowerVelocity() {
    return io_.flywheel_follower_velocity_;
  }

  /**
   * Returns the leader flywheel motor output current.
   *
   * @return Current draw in amps.
   */
  public double getFlywheelLeaderCurrent() {
    return io_.current_flywheel_leader_;
  }

  /**
   * Returns the follower flywheel motor output current.
   *
   * @return Current draw in amps.
   */
  public double getFlywheelFollowerCurrent() {
    return io_.current_flywheel_follower_;
  }

  // Hood
  /**
   * Sets the hood open-loop output.
   *
   * @param percent Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setHoodPercent(double percent) {
    hood_output_type_ = OutputType.PERCENT;
    io_.hood_demand_ = percent;
  }

  public void setHoodBrake() {
    hood_output_type_ = OutputType.BRAKE;
  }

  public void setRotationBrake() {
    rotation_output_type_ = OutputType.BRAKE;
  }

  /**
   * Sets the hood target angle using closed-loop position control.
   *
   * @param position Hood angle in degrees, clamped to the configured hood soft limits.
   */
  public void setHoodPosition(double position) {
    hood_output_type_ = OutputType.POSITION;
    io_.hood_target_ =
        MathUtil.clamp(position, Constants.kMinHoodPosition, Constants.kMaxHoodPosition);
  }

  public void stopHood() {
    setHoodPercent(0);
  }

  /**
   * Returns whether the hood has reached its closed-loop target.
   *
   * @return {@code true} when the hood is in position mode and within the configured tolerance.
   */
  public boolean isHoodAtTarget() {
    return hood_output_type_ == OutputType.POSITION && hood_pid_.atSetpoint();
  }

  /**
   * Returns the measured hood angle.
   *
   * @return Hood position in degrees.
   */
  public double getHoodPosition() {
    return io_.hood_position_;
  }

  /**
   * Returns the current hood closed-loop target.
   *
   * @return Hood target angle in degrees.
   */
  public double getHoodTarget() {
    return io_.hood_target_;
  }

  /**
   * Returns the hood motor output current.
   *
   * @return Current draw in amps.
   */
  public double getHoodCurrent() {
    return io_.current_hood_;
  }

  /**
   * Returns the currently applied hood output after clamping and soft-limit handling.
   *
   * @return Motor output in the range {@code [-1.0, 1.0]}.
   */
  public double getHoodPercent() {
    return io_.hood_output_;
  }

  // Rotation
  /**
   * Sets the turret rotation open-loop output.
   *
   * @param percent Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setRotationPercent(double percent) {
    rotation_output_type_ = OutputType.PERCENT;
    io_.rotation_demand_ = percent;
  }

  /**
   * Sets the turret target angle using closed-loop position control.
   *
   * @param position Turret angle in degrees, clamped to the configured rotation soft limits.
   */
  public void setRotationPosition(double position) {
    rotation_output_type_ = OutputType.POSITION;
    io_.rotation_target_ =
        MathUtil.clamp(position, Constants.kMinRotationPosition, Constants.kMaxRotationPosition);
  }

  public void stopRotation() {
    setRotationPercent(0);
  }

  /**
   * Returns whether the turret has reached its closed-loop target.
   *
   * @return {@code true} when the turret is in position mode and within the configured tolerance.
   */
  public boolean isRotationAtTarget() {
    return rotation_output_type_ == OutputType.POSITION && rotation_pid_.atSetpoint();
  }

  /**
   * Returns the measured turret angle.
   *
   * @return Turret position in degrees.
   */
  public double getRotationPosition() {
    return io_.rotation_position_;
  }

  /**
   * Returns the current turret closed-loop target.
   *
   * @return Turret target angle in degrees.
   */
  public double getRotationTarget() {
    return io_.rotation_target_;
  }

  /**
   * Returns the turret rotation motor output current.
   *
   * @return Current draw in amps.
   */
  public double getRotationCurrent() {
    return io_.current_rotation_;
  }

  /**
   * Returns the currently applied turret rotation output after clamping and soft-limit handling.
   *
   * @return Motor output in the range {@code [-1.0, 1.0]}.
   */
  public double getRotationPercent() {
    return io_.rotation_output_;
  }

  // Helpers
  public void stopAll() {
    stopFlywheel();
    stopHood();
    stopRotation();
  }

  private enum OutputType {
    PERCENT, POSITION, BRAKE
  }

  public static class PeriodicIO {
    // Inputs
    double flywheel_leader_velocity_;
    double flywheel_follower_velocity_;
    double hood_position_;
    double rotation_position_;
    double current_flywheel_leader_;
    double current_flywheel_follower_;
    double current_hood_;
    double current_rotation_;

    // Outputs
    double flywheel_demand_;
    double flywheel_output_;
    double hood_demand_;
    double hood_output_;
    double rotation_demand_;
    double rotation_output_;
    double hood_target_;
    double rotation_target_;
  }

  public static class Constants {
    public static final double kVoltageCompensation = 12.0;

    // CAN IDs
    public static final int kFlywheelLeaderId = 14;
    public static final int kFlywheelFollowerId = 15;
    public static final int kHoodId = 16;
    public static final int kRotationId = 13;

    // Flywheel
    public static final boolean kFlywheelLeaderInverted = false;
    public static final boolean kFlywheelFollowerOpposesLeader = true;
    public static final int kFlywheelCurrentLimit = 30; // Keep between [30, 40]
    public static final double kFlywheelGearRatio = 1.0; // CHANGE
    public static final double kMaxFlywheelOutput = 1.0;

    // Hood
    public static final boolean kHoodInverted = false;
    public static final int kHoodCurrentLimit = 30; // Keep between [20, 40]
    public static final double kHoodGearRatio = 10.0; // CHANGE
    public static final double kHoodStartingPosition = 0.0;
    public static final double kMinHoodPosition = 0.0;
    public static final double kMaxHoodPosition = 50.0;
    public static final double kHoodP = 0.05;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;
    public static final double kHoodTolerance = 0.5;
    public static final double kMaxHoodOutput = 0.4;

    // Rotation
    public static final boolean kRotationInverted = false;
    public static final int kRotationCurrentLimit = 20; // Keep between [20, 40]
    public static final double kRotationGearRatio = 10.0; // CHANGE
    public static final double kRotationStartingPosition = 0.0;
    public static final double kMinRotationPosition = -10.0;
    public static final double kMaxRotationPosition = 10.0;
    public static final double kRotationP = 0.05;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;
    public static final double kRotationTolerance = 0.5;
    public static final double kMaxRotationOutput = 0.5;
    public static final double kEncoderOffset = 0.0; // CHANGE THIS
  }
}
