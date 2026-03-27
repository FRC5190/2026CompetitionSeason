package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


public class ShootAuto {

  // public static Command shoot(Turret turret, Indexer indexer, VisionSubsystem
  // vision) {
  // return Commands.sequence(
  // Commands.runOnce(() -> turret.setFlywheelPercent(0.7)),

  // Commands.parallel(
  // Commands.runOnce(() -> turret.setHoodPercent(0.15)),
  // Commands.waitSeconds(0.5) // Wait for hood to raise
  // ),

  // Commands.waitSeconds(1.5),

  // Commands.runOnce(() -> indexer.setPercent(-0.6)),

  // Commands.waitSeconds(2.0), // Adjust this time based on how long feeding
  // takes

  // Commands.runOnce(() -> {
  // indexer.stop();
  // turret.stopFlywheel();
  // turret.setHoodBrake(); // Use brake instead of stop
  // })
  // );
  // }

  public static Command shoot(Turret turret, Indexer indexer, Intake intake) {
    return Commands.sequence(

        Commands.runOnce(() -> intake.setExtensionPosition(1.3)),
        Commands.parallel(Commands.runOnce(() -> turret.setFlywheelPercent(0.5)),
            Commands.waitSeconds(1)),
        Commands.parallel(Commands.run(() -> turret.setFlywheelPercent(0.45), turret),
            // Commands.runOnce(() -> turret.setHoodPercent(0.15)),
            Commands.run(() -> indexer.setPercent(-0.4), indexer)).withTimeout(10),
        Commands.runOnce(() -> {
          indexer.stop();
          turret.stopFlywheel();
          turret.stopHood();
        }));
  }
}
