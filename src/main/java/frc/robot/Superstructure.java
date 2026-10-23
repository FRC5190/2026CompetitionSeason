package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class Superstructure {
  private final Intake intake;
  private final Indexer indexer;
  private final Turret turret;

  public Superstructure(Intake intake, Indexer indexer, Turret turret) {
    this.intake = intake;
    this.indexer = indexer;
    this.turret = turret;
  }

  // --- Intake ---
  // Extension Position Command
  public Command setExtensionPosition(double position) {
    return new RunCommand(() -> intake.setExtensionPosition(position), intake)
        .until(() -> intake.isExtensionAtTarget())
        .andThen(new InstantCommand(() -> intake.stopExtension())).withTimeout(Seconds.of(2));
  }

  // public Command extendIntakeRoll(double percent) {
  // return new ParallelCommandGroup(jogRoller(percent),
  // setExtensionPosition(3.5));
  // }

  // public Command indexerAndShooter(double percent) {
  // return new ParallelCommandGroup(jogIndexer(-1 * percent),
  // runFlywheel(percent));
  // }

  // Roller jog
  public Command jogRoller(double percent) {
    return new StartEndCommand(() -> intake.setRollerPercent(percent), () -> intake.stopRoller(),
        intake);
  }

  // Extension jog
  public Command jogExtension(double percent) {
    return new StartEndCommand(() -> intake.setExtensionPercent(percent),
        () -> intake.stopExtension(), intake);
  }

  // Indexer jog
  public Command jogIndexer(double percent) {
    return new StartEndCommand(() -> indexer.setPercent(percent), () -> indexer.stop(), indexer);
  }

  public Command runIndexerWithJamRecovery(double requestedPercent) {
    final Timer timer = new Timer();
    return new FunctionalCommand(
        () -> {
          indexer.setPercent(requestedPercent);
          indexer.resetReverseCheckTimer();
        },
        () -> {
          double elapsedSeconds = timer.get();
          timer.reset();
          timer.start();
          indexer.updateJamRecovery(requestedPercent, elapsedSeconds);
        },
        interrupted -> {
          indexer.stop();
        },
        () -> false,
        indexer);
  }

  // --- Flywheel ---
  public Command runFlywheel(double percent) {
    return new StartEndCommand(() -> turret.setFlywheelPercent(percent),
        () -> turret.stopFlywheel(), turret);
  }

  // --- Hood ---
  public Command jogHoodUp(double percent) {
    return new StartEndCommand(() -> turret.setHoodPercent(percent), () -> turret.setHoodPercent(0.025),
        turret);
  }

  // public Command jogHoodDown() {
  // return new StartEndCommand(() ->
  // turret.setHoodPercent(-Constants.kHoodJogPercent),
  // () -> turret.stopHood(), turret);
  // }

  public Command setHoodPosition(double position) {
    return new RunCommand(() -> turret.setHoodPosition(position), turret)
        .until(() -> turret.isHoodAtTarget())
        .andThen(new InstantCommand(() -> turret.stopHood())).withTimeout(Seconds.of(2));
  }

  // --- Rotation ---
  // public Command jogRotationLeft() {
  // return new StartEndCommand(() ->
  // turret.setRotationPercent(-Constants.kRotationJogPercent),
  // () -> turret.stopRotation(), turret);
  // }

  public Command jogRotationRight(double percent) {
    return new StartEndCommand(() -> turret.setRotationPercent(percent),
        () -> turret.stopRotation(), turret);
  }

  public Command setRotationPosition(double position) {
    return new RunCommand(() -> turret.setRotationPosition(position), turret)
        .until(() -> turret.isRotationAtTarget());
  }

  public Command autoAim(java.util.function.DoubleSupplier targetAngleDegrees,
      java.util.function.BooleanSupplier targetVisible) {
    return new RunCommand(() -> {
      if (targetVisible.getAsBoolean()) {
        turret.setRotationPosition(targetAngleDegrees.getAsDouble());
      } else {
        turret.stopRotation();
      }
    }, turret);
  }

  public static class Constants {
    public static final double kHoodJogPercent = 0.2;
    public static final double kRotationJogPercent = 0.3;
  }

}
