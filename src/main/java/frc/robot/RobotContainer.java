package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootAuto;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final String kDefaultAutonomous = "ShootOnlyAuto";

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

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
          () -> m_driverController.getLeftX() * -2.0)
      .withControllerRotationAxis(m_driverController::getRightX)
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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Driver controls
    m_driverController.b().and(new Trigger(() -> drivebase.seesTag(26))).
        whileTrue(drivebase.alignToOffset(-0.5, 0.5, 0, 26));
    m_driverController.povLeft().onTrue(Commands.runOnce(drivebase::resetGyro, drivebase));
    m_driverController.leftTrigger().whileTrue(superstructure.jogIndexer(-0.4));
    m_driverController.rightTrigger().whileTrue(superstructure.runFlywheel(-0.45));
    m_driverController.leftBumper().whileTrue(superstructure.jogIndexer(0.4));
    m_driverController.rightBumper().whileTrue(superstructure.runFlywheel(0.45));
    m_driverController.a().onTrue(superstructure.setExtensionPosition(1.7));

    // Operator controls
    m_operatorController.y().whileTrue(superstructure.jogHoodUp(0.45));
    m_operatorController.a().whileTrue(superstructure.jogHoodUp(-0.02));
    m_operatorController.b().onTrue(superstructure.setHoodPosition(30));
    m_operatorController.rightBumper().whileTrue(superstructure.jogExtension(0.25));
    m_operatorController.leftBumper().whileTrue(superstructure.jogExtension(-0.7));
    m_operatorController.rightTrigger().whileTrue(superstructure.jogRoller(0.75));
    m_operatorController.leftTrigger().whileTrue(superstructure.jogRoller(-0.75));
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
