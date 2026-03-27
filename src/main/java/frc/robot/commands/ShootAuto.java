package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Indexer;


public class ShootAuto {
  private static final double kShootingHoodPositionDeg = 30.0;
  private static final double kFlywheelSpinupPercent = 0.45;
  private static final double kIndexerFeedPercent = -0.4;
  private static final double kHoodSettleTimeoutSec = 2.0;
  private static final double kFlywheelSpinupTimeSec = 1.0;
  private static final double kFeedTimeSec = 2.0;

  public static Command shoot(Turret turret, Indexer indexer) {
    return Commands.sequence(
        Commands.runOnce(() -> turret.setHoodPosition(kShootingHoodPositionDeg), turret),
        Commands.runOnce(() -> turret.setFlywheelPercent(kFlywheelSpinupPercent), turret),
        Commands.parallel(
            Commands.waitUntil(turret::isHoodAtTarget).withTimeout(kHoodSettleTimeoutSec),
            Commands.waitSeconds(kFlywheelSpinupTimeSec)),
        Commands.run(() -> indexer.setPercent(kIndexerFeedPercent), indexer)
            .withTimeout(kFeedTimeSec),
        Commands.runOnce(() -> {
          indexer.stop();
          turret.stopFlywheel();
          turret.setHoodBrake();
        }));
  }
}
