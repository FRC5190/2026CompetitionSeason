package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.Superstructure;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionSubsystem;


public class ShootAuto {

  // static Intake intake = new Intake();
  // static Indexer indexer = new Indexer();
  // static Turret turret = new Turret();

  //static Superstructure superstructure = new Superstructure(intake, indexer, turret);

  // public static Command shoot(Turret turret, Indexer indexer, VisionSubsystem
  // vision) {
  // return Commands.sequence(
  // // Step 1: Start flywheel
  // Commands.runOnce(() -> turret.setFlywheelPercent(0.7)),

  // // Step 2: Raise hood while flywheel spins up
  // Commands.parallel(
  // Commands.runOnce(() -> turret.setHoodPercent(0.15)),
  // Commands.waitSeconds(0.5) // Wait for hood to raise
  // ),

  // // Step 3: Wait for flywheel to spin up
  // Commands.waitSeconds(1.5),

  // // Step 4: Start feeding
  // Commands.runOnce(() -> indexer.setPercent(-0.6)),

  // // Step 5: Feed for duration
  // Commands.waitSeconds(2.0), // Adjust this time based on how long feeding
  // takes

  // // Step 6: Stop everything
  // Commands.runOnce(() -> {
  // indexer.stop();
  // turret.stopFlywheel();
  // turret.setHoodBrake(); // Use brake instead of stop
  // })
  // );
  // }

  public static Command shoot(Superstructure superstructure, VisionSubsystem vision) {
    
  
    return Commands.sequence(
        // Step 1: Start flywheel

        superstructure.setExtensionPosition(1.3),
        Commands.parallel(
          superstructure.runFlywheel(0.45)).withTimeout(3),
        Commands.parallel(
            superstructure.runFlywheel(0.45),
            // Commands.run(()-> turret.setFlywheelPercent(0.7), turret),
            superstructure.jogIndexer(0.4)).withTimeout(1),
        Commands.parallel(
            superstructure.runFlywheel(0.45),
            // Commands.run(()-> turret.setFlywheelPercent(0.7), turret),
            superstructure.jogIndexer(-0.4)).withTimeout(5),
        Commands.parallel(
            superstructure.runFlywheel(0.45),
            // Commands.run(()-> turret.setFlywheelPercent(0.7), turret),
            superstructure.jogIndexer(0.4)).withTimeout(1),
        Commands.parallel(
            superstructure.runFlywheel(0.45),
            // Commands.run(()-> turret.setFlywheelPercent(0.7), turret),
            superstructure.jogIndexer(-0.4)).withTimeout(5)
          

        // Commands.runOnce(() -> {
        //  // superstructure.intake_.jogExtension(0.0);
        //   superstructure.turret_.stopFlywheel();
        //   // superstructure.stopHood(); 
        // }
        );
  }
}