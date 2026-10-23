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
  private final SparkMax flywheelLeaderMotor;
  private final SparkMax flywheelFollowerMotor;
  private final RelativeEncoder flywheelLeaderEncoder;
  private final RelativeEncoder flywheelFollowerEncoder;

  // Hood motor
  private final SparkMax hoodMotor;
  private final RelativeEncoder hoodEncoder;
  private final PIDController hoodPid;

  // Rotation motor
  private final SparkMax rotationMotor;
  private final RelativeEncoder rotationEncoder;
  private final PIDController rotationPid;
  private final SparkAbsoluteEncoder absoluteRotationEncoder;

  // Periodic IO
  private final PeriodicIO periodicIO = new PeriodicIO();
  private ControlMode hoodControlMode = ControlMode.PERCENT;
  private ControlMode rotationControlMode = ControlMode.BRAKE;

  public Turret() {
    // Flywheel leader
    SparkMaxConfig flywheelLeaderConfig = new SparkMaxConfig();
    flywheelLeaderConfig.voltageCompensation(Constants.kVoltageCompensation);
    flywheelLeaderConfig.smartCurrentLimit(Constants.kFlywheelCurrentLimit);
    flywheelLeaderConfig.inverted(Constants.kFlywheelLeaderInverted);
    flywheelLeaderConfig.idleMode(IdleMode.kCoast);
    flywheelLeaderConfig.encoder.positionConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheelLeaderConfig.encoder.velocityConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheelLeaderMotor = new SparkMax(Constants.kFlywheelLeaderId, kBrushless);
    flywheelLeaderEncoder = flywheelLeaderMotor.getEncoder();

    flywheelLeaderMotor.configure(flywheelLeaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Flywheel follower
    SparkMaxConfig flywheelFollowerConfig = new SparkMaxConfig();
    flywheelFollowerConfig.voltageCompensation(Constants.kVoltageCompensation);
    flywheelFollowerConfig.smartCurrentLimit(Constants.kFlywheelCurrentLimit);
    flywheelFollowerConfig.idleMode(IdleMode.kCoast);
    flywheelFollowerConfig.encoder.positionConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheelFollowerConfig.encoder.velocityConversionFactor(1.0 / Constants.kFlywheelGearRatio);
    flywheelFollowerConfig.follow(Constants.kFlywheelLeaderId,
        Constants.kFlywheelFollowerOpposesLeader);

    flywheelFollowerMotor = new SparkMax(Constants.kFlywheelFollowerId, kBrushless);
    flywheelFollowerEncoder = flywheelFollowerMotor.getEncoder();
    flywheelFollowerMotor.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Hood
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.voltageCompensation(Constants.kVoltageCompensation);
    hoodConfig.smartCurrentLimit(Constants.kHoodCurrentLimit);
    hoodConfig.inverted(Constants.kHoodInverted);
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.encoder.positionConversionFactor(360.0 / Constants.kHoodGearRatio);
    hoodConfig.encoder.velocityConversionFactor((360.0 / Constants.kHoodGearRatio) / 60.0);

    hoodMotor = new SparkMax(Constants.kHoodId, kBrushless);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPosition(Constants.kHoodStartingPosition);

    // Rotation
    SparkMaxConfig rotationConfig = new SparkMaxConfig();
    rotationConfig.voltageCompensation(Constants.kVoltageCompensation);
    rotationConfig.smartCurrentLimit(Constants.kRotationCurrentLimit);
    rotationConfig.inverted(Constants.kRotationInverted);
    rotationConfig.idleMode(IdleMode.kBrake);
    rotationConfig.encoder.positionConversionFactor(360.0 / Constants.kRotationGearRatio);
    rotationConfig.encoder.velocityConversionFactor((360.0 / Constants.kRotationGearRatio) / 60.0);
    rotationConfig.apply(
        new AbsoluteEncoderConfig().apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2)
            .positionConversionFactor(2.0 * Math.PI) // rotations -> radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0) // rpm -> rad/s
            .zeroOffset(Constants.kEncoderOffset));

    rotationMotor = new SparkMax(Constants.kRotationId, kBrushless);
    rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    absoluteRotationEncoder = rotationMotor.getAbsoluteEncoder();

    rotationEncoder = rotationMotor.getEncoder();
    rotationEncoder.setPosition(Constants.kRotationStartingPosition);

    hoodPid = new PIDController(Constants.kHoodP, Constants.kHoodI, Constants.kHoodD);
    hoodPid.setTolerance(Constants.kHoodTolerance);

    rotationPid =
        new PIDController(Constants.kRotationP, Constants.kRotationI, Constants.kRotationD);
    rotationPid.setTolerance(Constants.kRotationTolerance);

    // Assign absolute encoder value to rotation encoder
    synchronizeRotationEncoderFromAbsolute();
  }

  @Override
  public void periodic() {
    // Read inputs
    periodicIO.flywheelLeaderVelocityRpm = flywheelLeaderEncoder.getVelocity();
    periodicIO.flywheelFollowerVelocityRpm = flywheelFollowerEncoder.getVelocity();
    periodicIO.hoodPositionDegrees = hoodEncoder.getPosition();
    periodicIO.rotationPositionDegrees = rotationEncoder.getPosition();

    periodicIO.flywheelLeaderCurrentAmps = flywheelLeaderMotor.getOutputCurrent();
    periodicIO.flywheelFollowerCurrentAmps = flywheelFollowerMotor.getOutputCurrent();
    periodicIO.hoodCurrentAmps = hoodMotor.getOutputCurrent();
    periodicIO.rotationCurrentAmps = rotationMotor.getOutputCurrent();

    // Flywheel outputs
    periodicIO.flywheelOutput = MathUtil.clamp(periodicIO.flywheelDemand, -Constants.kMaxFlywheelOutput,
        Constants.kMaxFlywheelOutput);

    // Hood outputs
    switch (hoodControlMode) {
      case POSITION:
        periodicIO.hoodOutput = MathUtil.clamp(hoodPid.calculate(periodicIO.hoodPositionDegrees, periodicIO.hoodTargetDegrees),
            -Constants.kMaxHoodOutput, Constants.kMaxHoodOutput);
        break;
      case PERCENT:
        periodicIO.hoodOutput =
            MathUtil.clamp(periodicIO.hoodDemand, -Constants.kMaxHoodOutput, Constants.kMaxHoodOutput);
        break;
      case BRAKE:
      default:
        hoodPid.setSetpoint(getHoodPosition());
        periodicIO.hoodOutput = MathUtil.clamp(hoodPid.calculate(periodicIO.hoodPositionDegrees), -0.9, 0.9);
    }

    // Rotation outputs
    switch (rotationControlMode) {
      case POSITION:
        periodicIO.rotationOutput =
            MathUtil.clamp(rotationPid.calculate(periodicIO.rotationPositionDegrees, periodicIO.rotationTargetDegrees),
                -Constants.kMaxRotationOutput, Constants.kMaxRotationOutput);
        break;
      case PERCENT:
        periodicIO.rotationOutput = MathUtil.clamp(periodicIO.rotationDemand, -Constants.kMaxRotationOutput,
            Constants.kMaxRotationOutput);
        break;
      case BRAKE:
      default:
        rotationPid.setSetpoint(getRotationPosition());
        periodicIO.rotationOutput = MathUtil.clamp(rotationPid.calculate(periodicIO.rotationPositionDegrees), -0.1, 0.1);
    }

    // Hood soft limits
    if (periodicIO.hoodPositionDegrees >= Constants.kMaxHoodPosition && periodicIO.hoodOutput > 0) {
      periodicIO.hoodOutput = 0;
    } else if (periodicIO.hoodPositionDegrees <= Constants.kMinHoodPosition && periodicIO.hoodOutput < 0) {
      periodicIO.hoodOutput = 0;
    }

    // Rotation soft limits
    if (periodicIO.rotationPositionDegrees >= Constants.kMaxRotationPosition && periodicIO.rotationOutput > 0) {
      periodicIO.rotationOutput = 0;
    } else if (periodicIO.rotationPositionDegrees <= Constants.kMinRotationPosition
        && periodicIO.rotationOutput < 0) {
      periodicIO.rotationOutput = 0;
    }

    // Write outputs
    flywheelLeaderMotor.set(periodicIO.flywheelOutput);
    hoodMotor.set(periodicIO.hoodOutput);
    rotationMotor.set(periodicIO.rotationOutput);
  }

  /**
   * Returns the turret absolute encoder angle in radians, normalized to {@code [-pi, pi)}.
   *
   * @return Absolute turret angle in radians.
   */
  public double getAbsoluteRotationRadians() {
    return absoluteRotationEncoder.getPosition();
  }

  /**
   * Synchronizes the relative turret rotation encoder to the absolute encoder reading.
   *
   * <p>
   * The relative encoder is stored in degrees, so the absolute encoder radians are converted before
   * writing.
   */
  public void synchronizeRotationEncoderFromAbsolute() {
    rotationEncoder.setPosition(Math.toDegrees(getAbsoluteRotationRadians()));
  }

  // Flywheel
  /**
   * Sets the flywheel open-loop output.
   *
   * @param percent Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setFlywheelPercent(double percent) {
    periodicIO.flywheelDemand = percent;
  }

  public void stopFlywheel() {
    periodicIO.flywheelDemand = 0;
  }

  /**
   * Returns the currently applied flywheel output after clamping.
   *
   * @return Motor output in the range {@code [-1.0, 1.0]}.
   */
  public double getFlywheelPercent() {
    return periodicIO.flywheelOutput;
  }

  /**
   * Returns the requested flywheel output before clamping.
   *
   * @return Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public double getFlywheelDemand() {
    return periodicIO.flywheelDemand;
  }

  /**
   * Returns the leader flywheel wheel speed.
   *
   * @return Flywheel velocity in wheel rotations per minute.
   */
  public double getFlywheelLeaderVelocity() {
    return periodicIO.flywheelLeaderVelocityRpm;
  }

  /**
   * Returns the follower flywheel wheel speed.
   *
   * @return Flywheel velocity in wheel rotations per minute.
   */
  public double getFlywheelFollowerVelocity() {
    return periodicIO.flywheelFollowerVelocityRpm;
  }

  /**
   * Returns the leader flywheel motor output current.
   *
   * @return Current draw in amps.
   */
  public double getFlywheelLeaderCurrent() {
    return periodicIO.flywheelLeaderCurrentAmps;
  }

  /**
   * Returns the follower flywheel motor output current.
   *
   * @return Current draw in amps.
   */
  public double getFlywheelFollowerCurrent() {
    return periodicIO.flywheelFollowerCurrentAmps;
  }

  // Hood
  /**
   * Sets the hood open-loop output.
   *
   * @param percent Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setHoodPercent(double percent) {
    hoodControlMode = ControlMode.PERCENT;
    periodicIO.hoodDemand = percent;
  }

  public void setHoodBrake() {
    hoodControlMode = ControlMode.BRAKE;
  }

  public void setRotationBrake() {
    rotationControlMode = ControlMode.BRAKE;
  }

  /**
   * Sets the hood target angle using closed-loop position control.
   *
   * @param position Hood angle in degrees, clamped to the configured hood soft limits.
   */
  public void setHoodPosition(double position) {
    hoodControlMode = ControlMode.POSITION;
    periodicIO.hoodTargetDegrees =
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
    return hoodControlMode == ControlMode.POSITION && hoodPid.atSetpoint();
  }

  /**
   * Returns the measured hood angle.
   *
   * @return Hood position in degrees.
   */
  public double getHoodPosition() {
    return periodicIO.hoodPositionDegrees;
  }

  /**
   * Returns the current hood closed-loop target.
   *
   * @return Hood target angle in degrees.
   */
  public double getHoodTarget() {
    return periodicIO.hoodTargetDegrees;
  }

  /**
   * Returns the hood motor output current.
   *
   * @return Current draw in amps.
   */
  public double getHoodCurrent() {
    return periodicIO.hoodCurrentAmps;
  }

  /**
   * Returns the currently applied hood output after clamping and soft-limit handling.
   *
   * @return Motor output in the range {@code [-1.0, 1.0]}.
   */
  public double getHoodPercent() {
    return periodicIO.hoodOutput;
  }

  // Rotation
  /**
   * Sets the turret rotation open-loop output.
   *
   * @param percent Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setRotationPercent(double percent) {
    rotationControlMode = ControlMode.PERCENT;
    periodicIO.rotationDemand = percent;
  }

  /**
   * Sets the turret target angle using closed-loop position control.
   *
   * @param position Turret angle in degrees, clamped to the configured rotation soft limits.
   */
  public void setRotationPosition(double position) {
    rotationControlMode = ControlMode.POSITION;
    periodicIO.rotationTargetDegrees =
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
    return rotationControlMode == ControlMode.POSITION && rotationPid.atSetpoint();
  }

  /**
   * Returns the measured turret angle.
   *
   * @return Turret position in degrees.
   */
  public double getRotationPosition() {
    return periodicIO.rotationPositionDegrees;
  }

  /**
   * Returns the current turret closed-loop target.
   *
   * @return Turret target angle in degrees.
   */
  public double getRotationTarget() {
    return periodicIO.rotationTargetDegrees;
  }

  /**
   * Returns the turret rotation motor output current.
   *
   * @return Current draw in amps.
   */
  public double getRotationCurrent() {
    return periodicIO.rotationCurrentAmps;
  }

  /**
   * Returns the currently applied turret rotation output after clamping and soft-limit handling.
   *
   * @return Motor output in the range {@code [-1.0, 1.0]}.
   */
  public double getRotationPercent() {
    return periodicIO.rotationOutput;
  }

  // Helpers
  public void stopAll() {
    stopFlywheel();
    stopHood();
    stopRotation();
  }

  private enum ControlMode {
    PERCENT, POSITION, BRAKE
  }

  public static class PeriodicIO {
    // Inputs
    double flywheelLeaderVelocityRpm;
    double flywheelFollowerVelocityRpm;
    double hoodPositionDegrees;
    double rotationPositionDegrees;
    double flywheelLeaderCurrentAmps;
    double flywheelFollowerCurrentAmps;
    double hoodCurrentAmps;
    double rotationCurrentAmps;

    // Outputs
    double flywheelDemand;
    double flywheelOutput;
    double hoodDemand;
    double hoodOutput;
    double rotationDemand;
    double rotationOutput;
    double hoodTargetDegrees;
    double rotationTargetDegrees;
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
