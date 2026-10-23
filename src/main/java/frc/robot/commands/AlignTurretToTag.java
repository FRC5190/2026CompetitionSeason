package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSubsystem;

public class AlignTurretToTag extends Command {

  private final Turret turret;
  private final VisionSubsystem vision;

  // how agressively the motor moves to position (tune later)
  private static final double kProportionalGain = 0.02;

  // acceptable error (degrees)
  private static final double kAlignmentToleranceDegrees = 1.0;

  public AlignTurretToTag(Turret turret, VisionSubsystem vision) {
    this.turret = turret;
    this.vision = vision;

    addRequirements(turret);
  }

  @Override
  public void execute() {

    if (!vision.hasTarget()) {
      turret.setRotationPercent(0.15); // slowly searches
      return;
    }

    double horizontalErrorDegrees = vision.getHorizontalOffsetDegrees();

    double rotationOutputPercent = horizontalErrorDegrees * kProportionalGain;

    turret.setRotationPercent(rotationOutputPercent);
  }

  @Override
  public boolean isFinished() {
    if (!vision.hasTarget())
      return false;

    return Math.abs(vision.getHorizontalOffsetDegrees()) < kAlignmentToleranceDegrees;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotation();
  }

}
