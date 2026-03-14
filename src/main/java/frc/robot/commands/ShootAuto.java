package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.VisionSubsystem;

public class ShootAuto {
    public static Command shoot(Turret turret, Indexer indexer){

        //double turretTarget = 0.0; //adjust this value based on where goal is

        return Commands.sequence(

            // align turret with AprilTag
            new AlignTurretToTag(turret, vision).withTimout(2.5),

            // spin flywheel
            Commands.runOnce(() -> turret.setFlywheelPercent(10.0)),
            Commands.waitSeconds(2.0),

            // feed fuel
            Commands.runOnce(() -> indexer.setPercent(0.8)),
            Commands.waitSeconds(1.5),

            // stop motors
            Commands.runOnce(() -> {
                indexer.stop();
                turret.stopFlywheel();
            })

        )
    }
}
