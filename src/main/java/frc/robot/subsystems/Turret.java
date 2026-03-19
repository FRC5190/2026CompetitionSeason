package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {

    // Flywheel motors (leader + follower)
    private final SparkMax flywheel_leader_;
    private final SparkMax flywheel_follower_;

    // Hood motor (adjusts shooting angle)
    private final SparkMax hood_;
    private final RelativeEncoder hood_encoder_;

    // Rotation motor (rotates entire turret)
    private final SparkMax rotation_;
    private final RelativeEncoder rotation_encoder_;

    private final PIDController hood_pid_;
    private final PIDController rotation_pid_;

    private final PeriodicIO io_ = new PeriodicIO();

    public enum ControlMode {
        MANUAL,
        PID
    }

    private ControlMode hood_mode_     = ControlMode.MANUAL;
    private ControlMode rotation_mode_ = ControlMode.MANUAL;

    public Turret() {

        // --- Flywheel Leader ---
        SparkMaxConfig flywheel_leader_config = new SparkMaxConfig();
        flywheel_leader_config.voltageCompensation(12);
        flywheel_leader_config.smartCurrentLimit(60);
        flywheel_leader_config.inverted(false);
        flywheel_leader_config.idleMode(IdleMode.kCoast); // Coast so flywheels spin down naturally

        flywheel_leader_ = new SparkMax(Constants.kFlywheelLeaderId, kBrushless);
        flywheel_leader_.configure(flywheel_leader_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Flywheel Follower (opposite direction) ---
        SparkMaxConfig flywheel_follower_config = new SparkMaxConfig();
        flywheel_follower_config.voltageCompensation(12);
        flywheel_follower_config.smartCurrentLimit(60);
        flywheel_follower_config.idleMode(IdleMode.kCoast);
        flywheel_follower_config.follow(Constants.kFlywheelLeaderId, true); // true = inverted relative to leader

        flywheel_follower_ = new SparkMax(Constants.kFlywheelFollowerId, kBrushless);
        flywheel_follower_.configure(flywheel_follower_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Hood Motor ---
        SparkMaxConfig hood_config = new SparkMaxConfig();
        hood_config.voltageCompensation(12);
        hood_config.smartCurrentLimit(20);
        hood_config.inverted(false);
        hood_config.idleMode(IdleMode.kBrake);

        hood_ = new SparkMax(Constants.kHoodId, kBrushless);
        hood_.configure(hood_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hood_encoder_ = hood_.getEncoder();
        hood_encoder_.setPosition(0);

        // --- Rotation Motor ---
        SparkMaxConfig rotation_config = new SparkMaxConfig();
        rotation_config.voltageCompensation(12);
        rotation_config.smartCurrentLimit(40);
        rotation_config.inverted(false);
        rotation_config.idleMode(IdleMode.kBrake);

        rotation_ = new SparkMax(Constants.kRotationId, kBrushless);
        rotation_.configure(rotation_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotation_encoder_ = rotation_.getEncoder();
        rotation_encoder_.setPosition(0);

        // --- PID Controllers ---
        hood_pid_     = new PIDController(Constants.kHoodP, Constants.kHoodI, Constants.kHoodD);
        rotation_pid_ = new PIDController(Constants.kRotationP, Constants.kRotationI, Constants.kRotationD);
        hood_pid_.setTolerance(Constants.kHoodTolerance);
        rotation_pid_.setTolerance(Constants.kRotationTolerance);
    }

    @Override
    public void periodic() {
        // Read inputs
        io_.hood_position_     = hood_encoder_.getPosition();
        io_.rotation_position_ = rotation_encoder_.getPosition();
        io_.current_flywheel_leader_   = flywheel_leader_.getOutputCurrent();
        io_.current_flywheel_follower_ = flywheel_follower_.getOutputCurrent();
        io_.current_hood_              = hood_.getOutputCurrent();
        io_.current_rotation_          = rotation_.getOutputCurrent();

        // Hood PID
        if (hood_mode_ == ControlMode.PID) {
            double pid_output = hood_pid_.calculate(io_.hood_position_, io_.hood_target_);
            io_.hood_demand_ = clamp(pid_output, -Constants.kMaxHoodOutput, Constants.kMaxHoodOutput);
        }

        // Rotation PID
        if (rotation_mode_ == ControlMode.PID) {
            double pid_output = rotation_pid_.calculate(io_.rotation_position_, io_.rotation_target_);
            io_.rotation_demand_ = clamp(pid_output, -Constants.kMaxRotationOutput, Constants.kMaxRotationOutput);
        }

        // Hood soft limits
        if (io_.hood_position_ >= Constants.kMaxHoodPosition && io_.hood_demand_ > 0) {
            io_.hood_demand_ = 0;
        } else if (io_.hood_position_ <= Constants.kMinHoodPosition && io_.hood_demand_ < 0) {
            io_.hood_demand_ = 0;
        }

        // Rotation soft limits
        if (io_.rotation_position_ >= Constants.kMaxRotationPosition && io_.rotation_demand_ > 0) {
            io_.rotation_demand_ = 0;
        } else if (io_.rotation_position_ <= Constants.kMinRotationPosition && io_.rotation_demand_ < 0) {
            io_.rotation_demand_ = 0;
        }

        // Write outputs
        flywheel_leader_.set(io_.flywheel_demand_);
        hood_.set(io_.hood_demand_);
        rotation_.set(io_.rotation_demand_);

        // SmartDashboard
        SmartDashboard.putNumber("Flywheel Output", io_.flywheel_demand_);
        SmartDashboard.putNumber("Hood Position", io_.hood_position_);
        SmartDashboard.putNumber("Hood Target", io_.hood_target_);
        SmartDashboard.putNumber("Rotation Position", io_.rotation_position_);
        SmartDashboard.putNumber("Rotation Target", io_.rotation_target_);
        SmartDashboard.putBoolean("Hood At Target", isHoodAtTarget());
        SmartDashboard.putBoolean("Rotation At Target", isRotationAtTarget());
    }

    // --- Flywheel ---
    public void setFlywheelPercent(double percent) {
        io_.flywheel_demand_ = percent;
    }

    public void stopFlywheel() {
        io_.flywheel_demand_ = 0;
    }

    // --- Hood ---
    public void setHoodPercent(double percent) {
        hood_mode_ = ControlMode.MANUAL;
        io_.hood_demand_ = percent;
    }

    public void setHoodPosition(double position) {
        hood_mode_ = ControlMode.PID;
        io_.hood_target_ = clamp(position, Constants.kMinHoodPosition, Constants.kMaxHoodPosition);
    }

    public void stopHood() {
        hood_mode_ = ControlMode.MANUAL;
        io_.hood_demand_ = 0;
    }

    public boolean isHoodAtTarget() {
        return hood_pid_.atSetpoint();
    }

    public double getHoodPosition() {
        return io_.hood_position_;
    }

    // --- Rotation ---
    public void setRotationPercent(double percent) {
        rotation_mode_ = ControlMode.MANUAL;
        io_.rotation_demand_ = percent;
    }

    public void setRotationPosition(double position) {
        rotation_mode_ = ControlMode.PID;
        io_.rotation_target_ = clamp(position, Constants.kMinRotationPosition, Constants.kMaxRotationPosition);
    }

    public void stopRotation() {
        rotation_mode_ = ControlMode.MANUAL;
        io_.rotation_demand_ = 0;
    }

    public boolean isRotationAtTarget() {
        return rotation_pid_.atSetpoint();
    }

    public double getRotationPosition() {
        return io_.rotation_position_;
    }

    // --- Helpers ---
    public void stopAll() {
        stopFlywheel();
        stopHood();
        stopRotation();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static class PeriodicIO {
        // Inputs
        double hood_position_;
        double rotation_position_;
        double current_flywheel_leader_;
        double current_flywheel_follower_;
        double current_hood_;
        double current_rotation_;

        // Outputs
        double flywheel_demand_;
        double hood_demand_;
        double rotation_demand_;
        double hood_target_;
        double rotation_target_;
    }

    public static class Constants {
        // CAN IDs
        public static final int kFlywheelLeaderId   = 14; // Set to your actual CAN ID
        public static final int kFlywheelFollowerId = 15; // Set to your actual CAN ID
        public static final int kHoodId             = 16; // Set to your actual CAN ID
        public static final int kRotationId         = 17; // Set to your actual CAN ID

        // Hood soft limits (in encoder rotations, tune to your robot)
        public static final double kMinHoodPosition = 0.0;
        public static final double kMaxHoodPosition = 20.0;

        // Rotation soft limits (in encoder rotations, tune to your robot)
        public static final double kMinRotationPosition = -10.0;
        public static final double kMaxRotationPosition =  10.0;

        // Hood PID
        public static final double kHoodP         = 0.05;
        public static final double kHoodI         = 0.0;
        public static final double kHoodD         = 0.0;
        public static final double kHoodTolerance = 0.5;
        public static final double kMaxHoodOutput = 0.4;

        // Rotation PID
        public static final double kRotationP         = 0.05;
        public static final double kRotationI         = 0.0;
        public static final double kRotationD         = 0.0;
        public static final double kRotationTolerance = 0.5;
        public static final double kMaxRotationOutput = 0.5;
    }
}