package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class Superstructure {
  public final Intake intake_;
  public final Indexer indexer_;
  public final Turret turret_;

  public Superstructure(Intake intake, Indexer indexer, Turret turret) {
    intake_ = intake;
    indexer_ = indexer;
    turret_ = turret;
  }

  // --- Intake ---
  // Extension Position Command
  public Command setExtensionPosition(double position) {
    return new RunCommand(() -> intake_.setExtensionPosition(position), intake_)
        .until(() -> intake_.isExtensionAtTarget())
        .andThen(new InstantCommand(() -> intake_.stopExtension()))
        .withTimeout(Seconds.of(2));
  }

  // Roller jog
  public Command jogRoller(double percent) {
    return new StartEndCommand(() -> intake_.setRollerPercent(percent), () -> intake_.stopRoller(),
        intake_);
  }

  // Extension jog
  public Command jogExtension(double percent) {
    return new StartEndCommand(() -> intake_.setExtensionPercent(percent),
        () -> intake_.stopExtension(), intake_);
  }

  // Indexer jog
  public Command jogIndexer(double percent) {
    return new StartEndCommand(() -> indexer_.setPercent(percent), () -> indexer_.stop(), indexer_);
  }

  // --- Flywheel ---
  public Command runFlywheel(double percent) {
    return new StartEndCommand(() -> turret_.setFlywheelPercent(percent),
        () -> turret_.stopFlywheel(), turret_);
  }

  // --- Hood ---
  public Command jogHoodUp() {
    return new StartEndCommand(() -> turret_.setHoodPercent(Constants.kHoodJogPercent),
        () -> turret_.stopHood(), turret_);
  }

  public Command jogHoodDown() {
    return new StartEndCommand(() -> turret_.setHoodPercent(-Constants.kHoodJogPercent),
        () -> turret_.stopHood(), turret_);
  }

  public Command setHoodPosition(double position) {
    return new RunCommand(() -> turret_.setHoodPosition(position), turret_)
        .until(() -> turret_.isHoodAtTarget());
  }

  // --- Rotation ---
  public Command jogRotationLeft() {
    return new StartEndCommand(() -> turret_.setRotationPercent(-Constants.kRotationJogPercent),
        () -> turret_.stopRotation(), turret_);
  }

  public Command jogRotationRight() {
    return new StartEndCommand(() -> turret_.setRotationPercent(Constants.kRotationJogPercent),
        () -> turret_.stopRotation(), turret_);
  }

  public Command setRotationPosition(double position) {
    return new RunCommand(() -> turret_.setRotationPosition(position), turret_)
        .until(() -> turret_.isRotationAtTarget());
  }

  public Command autoAim(java.util.function.DoubleSupplier visionTarget,
      java.util.function.BooleanSupplier hasTarget) {
    return new RunCommand(() -> {
      if (hasTarget.getAsBoolean()) {
        turret_.setRotationPosition(visionTarget.getAsDouble());
      } else {
        turret_.stopRotation();
      }
    }, turret_);
  }

  public static class Constants {
    public static final double kHoodJogPercent = 0.2;
    public static final double kRotationJogPercent = 0.3;
  }

}
