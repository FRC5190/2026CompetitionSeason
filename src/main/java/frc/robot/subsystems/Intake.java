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
  private final SparkMax rollerMotor;
  private final SparkMax extensionMotor;

  // Intake Encoder
  private final RelativeEncoder extensionEncoder;
  private final RelativeEncoder rollerEncoder;

  private final PIDController extensionPid;

  // Periodic IO
  private ExtensionControlMode extensionControlMode = ExtensionControlMode.PERCENT;
  private final PeriodicIO periodicIO = new PeriodicIO();

  public Intake() {
    // Roller
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(12);
    rollerConfig.inverted(false);
    rollerConfig.idleMode(IdleMode.kBrake);

    rollerMotor = new SparkMax(Constants.kRollerId, kBrushless);
    rollerEncoder = rollerMotor.getEncoder();


    // Extension
    SparkMaxConfig extensionConfig = new SparkMaxConfig();
    extensionConfig.voltageCompensation(12);
    extensionConfig.inverted(false);
    extensionConfig.idleMode(IdleMode.kBrake);
    extensionConfig.encoder.positionConversionFactor(1.0 / Constants.kExtensionGearRatio);
    extensionConfig.encoder.velocityConversionFactor(1.0 / Constants.kExtensionGearRatio);

    extensionMotor = new SparkMax(Constants.kExtensionId, kBrushless);
    extensionEncoder = extensionMotor.getEncoder();

    // Safety
    rollerConfig.smartCurrentLimit(Constants.kRollerCurrentLimit);
    extensionConfig.smartCurrentLimit(Constants.kExtensionCurrentLimit);
    extensionConfig.softLimit.forwardSoftLimit(Constants.kMaxExtensionPosition); // Double check
                                                                                  // this
    extensionConfig.softLimit.reverseSoftLimit(Constants.kMinExtensionPosition); // Double check
                                                                                  // this

    // Configuration

    extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    extensionPid = new PIDController(Constants.kExtensionP, 0.0, Constants.kExtensionD);
    extensionPid.setTolerance(Constants.kExtensionTolerance);
  }

  @Override
  public void periodic() {
    // Read inputs
    periodicIO.rollerCurrentAmps = rollerMotor.getOutputCurrent();
    periodicIO.extensionCurrentAmps = extensionMotor.getOutputCurrent();

    periodicIO.extensionPosition = extensionEncoder.getPosition();
    periodicIO.rollerVelocityRpm = rollerEncoder.getVelocity();

    // Roller Outputs
    rollerMotor.set(periodicIO.rollerDemand);

    // Extension Outputs
    switch (extensionControlMode) {
      case POSITION:
        extensionPid.setSetpoint(periodicIO.extensionTarget);
        double output = MathUtil.clamp(
            extensionPid.calculate(periodicIO.extensionPosition, periodicIO.extensionTarget), -0.2,
            0.2);
        periodicIO.extensionDemand = output;

        extensionMotor.set(periodicIO.extensionDemand);
        break;
      case BRAKE:
        periodicIO.extensionDemand = 0;

        extensionMotor.set(periodicIO.extensionDemand);
        break;
      case PERCENT:
        extensionMotor.set(periodicIO.extensionDemand);
        break;
    }
  }

  /**
   * Sets the intake roller open-loop output.
   *
   * @param value Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setRollerPercent(double value) {
    periodicIO.rollerDemand = value;
  }

  /**
   * Sets the extension motor open-loop output.
   *
   * @param value Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public void setExtensionPercent(double value) {
    extensionControlMode = ExtensionControlMode.PERCENT;
    periodicIO.extensionDemand = value;
  }

  /**
   * Sets the extension target using closed-loop position control.
   *
   * @param position Extension position in output-shaft rotations, clamped to the configured soft
   *        limits.
   */
  public void setExtensionPosition(double position) {
    extensionControlMode = ExtensionControlMode.POSITION;
    periodicIO.extensionTarget =
        MathUtil.clamp(position, Constants.kMinExtensionPosition, Constants.kMaxExtensionPosition);
  }

  /**
   * Holds the extension at its current measured position using closed-loop brake control.
   */
  public void setExtensionBrake() {
    setExtensionBrake(periodicIO.extensionPosition);
  }

  /**
   * Holds the extension at a specific position using closed-loop brake control.
   *
   * @param position Extension position in output-shaft rotations, clamped to the configured soft
   *        limits.
   */
  public void setExtensionBrake(double position) {
    extensionControlMode = ExtensionControlMode.BRAKE;
    periodicIO.extensionTarget =
        MathUtil.clamp(position, Constants.kMinExtensionPosition, Constants.kMaxExtensionPosition);
  }

  /** Stop all motors */
  public void stopAll() {
    periodicIO.rollerDemand = 0;
    periodicIO.extensionDemand = 0;
  }

  /** Stop only the roller */
  public void stopRoller() {
    periodicIO.rollerDemand = 0;
  }

  /** Stop only the extension motor */
  public void stopExtension() {
    extensionControlMode = ExtensionControlMode.PERCENT;
    periodicIO.extensionDemand = 0;
  }

  /**
   * Returns the intake roller motor output current.
   *
   * @return Current draw in amps.
   */
  public double getRollerCurrent() {
    return periodicIO.rollerCurrentAmps;
  }

  /**
   * Returns the extension motor output current.
   *
   * @return Current draw in amps.
   */
  public double getExtensionCurrent() {
    return periodicIO.extensionCurrentAmps;
  }

  /**
   * Returns the measured extension position.
   *
   * @return Extension position in output-shaft rotations.
   */
  public double getExtensionPosition() {
    return periodicIO.extensionPosition;
  }

  /**
   * Returns the measured intake roller speed.
   *
   * @return Roller velocity in motor rotations per minute.
   */
  public double getRollerVelocity() {
    return periodicIO.rollerVelocityRpm;
  }

  /**
   * Returns the requested roller output.
   *
   * @return Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public double getRollerDemand() {
    return periodicIO.rollerDemand;
  }

  /**
   * Returns the currently requested extension output.
   *
   * @return Motor output demand in the range {@code [-1.0, 1.0]}.
   */
  public double getExtensionDemand() {
    return periodicIO.extensionDemand;
  }

  /**
   * Returns the current closed-loop extension target.
   *
   * @return Extension target in output-shaft rotations.
   */
  public double getExtensionTarget() {
    return periodicIO.extensionTarget;
  }

  /**
   * Returns whether the extension has reached its active closed-loop target.
   *
   * @return {@code true} when the extension is in distance or brake mode and within tolerance.
   */
  public boolean isExtensionAtTarget() {
    return extensionControlMode != ExtensionControlMode.PERCENT && extensionPid.atSetpoint();
  }

  public boolean isExtensionNearTarget() {
    return Math.abs(periodicIO.extensionTarget - periodicIO.extensionPosition)
        <= Constants.kExtensionTolerance;
  }

  private enum ExtensionControlMode {
    PERCENT, POSITION, BRAKE
  }

  public static class PeriodicIO {
    // Inputs
    double rollerCurrentAmps;
    double extensionCurrentAmps;
    double extensionPosition;
    double rollerVelocityRpm;

    // Outputs
    double rollerDemand;
    double extensionDemand;
    double extensionTarget;
  }

  public static class Constants {
    // Extension Constants
    public static final int kExtensionId = 9;
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
    public static final int kRollerId = 10;

    public static final int kRollerCurrentLimit = 30; // Keep between [<20, 40] MAX 40
  }
}
