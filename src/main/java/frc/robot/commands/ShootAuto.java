package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.VisionSubsystem;

public class ShootAuto {
  public static Command shoot(Turret turret, Indexer indexer, VisionSubsystem vision) {

    // double turretTarget = 0.0; //adjust this value based on where goal is

    return Commands.parallel(

        // align turret with AprilTag
        // new AlignTurretToTag(turret, vision).withTimeout(2.5),

        // spin flywheel
        Commands.runOnce(() -> turret.setFlywheelPercent(0.7)), Commands.waitSeconds(13),

        // feed fuel
        Commands.runOnce(() -> indexer.setPercent(-0.6)), Commands.waitSeconds(15),

        Commands.runOnce(() -> turret.setHoodPercent(0.15)), Commands.waitSeconds(0.5),



        // stop motors
        Commands.runOnce(() -> {
          indexer.stop();
          turret.stopFlywheel();
          turret.stopHood();
        })

    );
  }
}
