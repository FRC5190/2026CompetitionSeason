package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSubsystem;

public class AlignTurretToTag extends Command{

    private final Turret turret;
    private final VisionSubsystem vision;

    // how agressively the motor moves to position (tune later)
    private static final double kP = 0.02;

    // acceptable error (degrees)
    private static final double tolerance = 1.0;

    public AlignTurretToTag(Turret turret, VisionSubsystem vision) {
        this.turret = turret;
        this.vision = vision;

        addRequirements(turret);
    }

    @Override
    public void execute() {

        if (!vision.hasTag()) {
            turret.setRotationPercent(0.15); // slowly searches
            return;
        }

        double error = vision.getTX();

        double output = error * kP;

        turret.setRotationPercent(output);
    }

    @Override
    public boolean isFinished() {
        if (!vision.hasTag()) return false;

        return Math.abs(vision.getTX()) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopRotation();
    }
    
}
