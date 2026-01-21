

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;

public class intake extends SubsystemBase{
  private final SparkMax leader_left_;
  
  private final SparkMaxConfig leader_left_config_ = new SparkMaxConfig();
  private final SparkMax leader_right_;
  private final SparkMaxConfig leader_right_config_ = new SparkMaxConfig();

  private final PeriodicIO io_ = new PeriodicIO();

  private final PIDController controller_ = new PIDController(Constants.kP, 0, 0);

  // Constructor
  public intake() {
    // Initialize motor controllers
    leader_left_ = new SparkMax(Constants.kLeaderLeftId, kBrushless);
    leader_left_.configure(leader_left_config_, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader_left_config_.voltageCompensation(12);
    leader_left_config_.smartCurrentLimit(20);
    leader_left_config_.inverted(true);
    leader_left_config_.idleMode(IdleMode.kBrake);

    leader_right_ = new SparkMax(Constants.kLeaderRightId, kBrushless);
    leader_right_.configure(leader_left_config_, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader_right_config_.voltageCompensation(12);
    leader_right_config_.smartCurrentLimit(20);
    leader_right_config_.inverted(true);
    leader_right_config_.idleMode(IdleMode.kBrake);
    
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.current_left_ = leader_left_.getOutputCurrent();
    //io_.current_right_ = leader_right_.getOutputCurrent();
    leader_left_.set(io_.left_demand);
    //leader_right_.set(io_.right_demand);
  }

  public void setPercent (double value) {
    io_.left_demand = value;
    io_.right_demand = value;
  }

  public void stopMotor() {
    io_.left_demand = 0;
    io_.right_demand = 0;
  }

  public double getLeftOutputCurrent() {
    return io_.current_left_;
  }

  public double getRightOutputCurrent() {
    return io_.current_right_;
  }

  public static class PeriodicIO {
    // Input
    double current_left_;
    double current_right_;

    // Output
    double left_demand;
    double right_demand;
  }

  public static class Constants {
    //Motor Controllers
    public static final int kLeaderLeftId = 11;
    public static final int kLeaderRightId = 12;
    //public static final int kLeaderRightId = 1;

    // PID Constants
    public static final double kP = 0.8;
  }

}
