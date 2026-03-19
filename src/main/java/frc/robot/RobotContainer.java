package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public final SwerveSubsystem drivebase = new SwerveSubsystem();
  public final VisionSubsystem vision = new VisionSubsystem();
  public final Intake intake = new Intake();
  public final Indexer indexer = new Indexer();
  public final Turret turret = new Turret();

   // Blue alliance tag
    private final Pose2d blue_target_ = new Pose2d(new Translation2d(0.0, 5.5), new Rotation2d());

    // Red alliance tag
    private final Pose2d red_target_  = new Pose2d(new Translation2d(16.54, 5.5), new Rotation2d());

  public final Superstructure superstructure = new Superstructure(intake, indexer, turret);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
          () -> m_driverController.getLeftY() * -1,
          () -> m_driverController.getLeftX() * -1)
          .withControllerRotationAxis(m_driverController::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
          m_driverController::getRightY)
          .headingWhile(true);


  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // CHANGE TO ONTRUE FROM WHILETRUE LATER - latest
    // m_driverController.rightBumper().and(new Trigger(() -> drivebase.seesTag(10))).
    // whileTrue(drivebase.alignToOffset(1, 0.50, 180, 10));

    // m_driverController.leftBumper().and(new Trigger(() -> drivebase.seesTag(15))).
    // whileTrue(drivebase.alignToOffset(0.25, 0, 0, 15));

    // m_driverController.a().and(new Trigger(() -> drivebase.seesTag(26))).
    // whileTrue(drivebase.alignToOffset(-0.5, 0.5, 0, 26));

    // m_driverController.y().and(new Trigger(() -> drivebase.seesTag(31))).
    // whileTrue(drivebase.alignToOffset(0.25, 0, 180, 31));

    m_driverController.a().whileTrue(superstructure.jogRoller(0.6));
    m_driverController.b().whileTrue(superstructure.jogRoller(-0.6));

    m_driverController.rightBumper().whileTrue(superstructure.jogExtension(0.2));
    m_driverController.leftBumper().whileTrue(superstructure.jogExtension(-0.2));

    //m_driverController.x().whileTrue(superstructure.jogIndexer(0.8));
    //m_driverController.y().whileTrue(superstructure.jogIndexer(-0.8));

    m_driverController.leftTrigger().whileTrue(new ShootOnTheMove(turret, drivebase, vision, getShootTarget()));

  }

  private Pose2d getShootTarget() {
    if (DriverStation.getAlliance().isPresent() && 
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return red_target_;
    }
    return blue_target_;
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("Taxi Auto Suhaan");

    Command pathCommand = drivebase.getAutonomousCommand("ScoreLeftSideAuto");
    Command shootCommand = ShootAuto.shoot(turret, indexer, vision);

    return Commands.sequence(pathCommand, shootCommand);
  }
}
