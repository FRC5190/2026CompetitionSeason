package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootAuto;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final String kDefaultAutonomous = "ShootOnlyAuto";

  public final SwerveSubsystem drivebase = new SwerveSubsystem();
  public final VisionSubsystem vision = new VisionSubsystem();
  public final Intake intake = new Intake();
  public final Indexer indexer = new Indexer();
  public final Turret turret = new Turret();
  private final SendableChooser<String> autonomousChooser = new SendableChooser<>();
  public final Superstructure superstructure = new Superstructure(intake, indexer, turret);

  private final CommandXboxController m_driverController = new CommandXboxController(1);
  private final CommandXboxController m_operatorController = new CommandXboxController(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureAutos();
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  // SENSITIVITY CHANGE THE 2.0 and -2.0 below
  SwerveInputStream driveAngularVelocity = SwerveInputStream
      .of(drivebase.getSwerveDrive(), () -> m_driverController.getLeftY() * -2.0,
          () -> m_driverController.getLeftX() * -2.0) // Left stick drives field-oriented translation.
      .withControllerRotationAxis(m_driverController::getRightX) // Right stick X rotates the robot.
      .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(false);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Driver: hold left trigger to extend the intake to the pickup position and run the intake roller.
    m_driverController.leftTrigger().onTrue(superstructure.setExtensionPosition(1.7));
    m_driverController.leftTrigger().whileTrue(superstructure.jogRoller(0.75));
    // Driver: tap left bumper to retract the intake extension.
    m_driverController.leftBumper().onTrue(superstructure.setExtensionPosition(0.0));
    // Driver: hold right trigger to feed the indexer toward the shooter.
    m_driverController.rightTrigger().whileTrue(superstructure.jogIndexer(-0.4));
    // Driver: hold right bumper to reverse the indexer.
    m_driverController.rightBumper().whileTrue(superstructure.jogIndexer(0.4));
    // Driver: tap D-pad left to zero the swerve gyro.
    m_driverController.povLeft().onTrue(Commands.runOnce(drivebase::resetGyro, drivebase));
    // Driver: hold D-pad up to manually extend the intake.
    m_driverController.povUp().whileTrue(superstructure.jogExtension(0.25));
    // Driver: hold D-pad down to manually retract the intake.
    m_driverController.povDown().whileTrue(superstructure.jogExtension(-0.7));
    // Driver: hold Y to raise the hood, then hold current angle on release.
    m_driverController.y().whileTrue(superstructure.jogHoodUp(0.45));
    // Driver: hold A to lower the hood, then hold current angle on release.
    m_driverController.a().whileTrue(superstructure.jogHoodUp(-0.02));

    // Operator: hold Y for the lowest flywheel speed preset.
    m_operatorController.y().whileTrue(superstructure.runFlywheel(0.25));
    // Operator: hold B for the second flywheel speed preset.
    m_operatorController.b().whileTrue(superstructure.runFlywheel(0.35));
    // Operator: hold A for the third flywheel speed preset.
    m_operatorController.a().whileTrue(superstructure.runFlywheel(0.45));
    // Operator: hold X for the highest flywheel speed preset.
    m_operatorController.x().whileTrue(superstructure.runFlywheel(0.55));
    // Operator: tap D-pad up for the lowest hood angle preset.
    m_operatorController.povUp().onTrue(superstructure.setHoodPosition(10.0));
    // Operator: tap D-pad right for the second hood angle preset.
    m_operatorController.povRight().onTrue(superstructure.setHoodPosition(20.0));
    // Operator: tap D-pad down for the third hood angle preset.
    m_operatorController.povDown().onTrue(superstructure.setHoodPosition(30.0));
    // Operator: tap D-pad left for the highest hood angle preset.
    m_operatorController.povLeft().onTrue(superstructure.setHoodPosition(40.0));
    // Operator: hold left trigger to run the intake roller inward.
    m_operatorController.leftTrigger().whileTrue(superstructure.jogRoller(0.75));
    // Operator: hold right trigger to run the intake roller in reverse.
    m_operatorController.rightTrigger().whileTrue(superstructure.jogRoller(-0.75));
    // Operator: hold left bumper to feed the indexer forward.
    m_operatorController.leftBumper().whileTrue(superstructure.jogIndexer(-0.4));
    // Operator: hold right bumper to reverse the indexer.
    m_operatorController.rightBumper().whileTrue(superstructure.jogIndexer(0.4));
  }

  private void configureAutos() {
    autonomousChooser.setDefaultOption(kDefaultAutonomous, kDefaultAutonomous);
    autonomousChooser.addOption("ScoreLeftSideAuto", "ScoreLeftSideAuto");
    autonomousChooser.addOption("ScoreRightSideAuto", "ScoreRightSideAuto");
    autonomousChooser.addOption("ScoreIntakeScore", "ScoreIntakeScore");
    autonomousChooser.addOption("Simple Auto", "Simple Auto");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoName = getResolvedAutoName();
    Command shootCommand = ShootAuto.shoot(turret, indexer);
    return Commands.sequence(shootCommand).withName("Auto: " + autoName);
  }

  public SendableChooser<String> getAutonomousChooser() {
    return autonomousChooser;
  }

  public String getSelectedAutoName() {
    return autonomousChooser.getSelected();
  }

  public String getResolvedAutoName() {
    String selectedAuto = autonomousChooser.getSelected();
    if (selectedAuto == null || selectedAuto.isBlank()) {
      return kDefaultAutonomous;
    }
    return selectedAuto;
  }
}
