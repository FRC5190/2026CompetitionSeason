package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose3d;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;
  private final AprilTagFieldLayout fieldLayout;
  private final Field2d field2d = new Field2d();
  private static final String kLL = "limelight"; // change if you renamed in LL UI



  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {
      
    try
      {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maximumSpeed, new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }

      setUpPathPlanner();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PoseEstimate llEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    if (llEst != null) {
      System.out.println("LL Tags: " + llEst.tagCount + " Pose: " + llEst.pose.getX() + ", " + llEst.pose.getY());
      if (llEst.tagCount > 0) {
        swerveDrive.addVisionMeasurement(llEst.pose, llEst.timestampSeconds);
        System.out.println("FUSED Vision");
      }      
    }

    field2d.setRobotPose(swerveDrive.getPose());
    //System.out.println("Swerve Pose: " + swerveDrive.getPose());
    SmartDashboard.putData("Field", field2d);


    Pose2d pose = swerveDrive.getPose();
    System.out.println("Robot POSE: X= " + swerveDrive.getPose());
    System.out.println("test1");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    // TODO Auto-generated method stub

    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  
  
  public void setUpPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      final boolean enableFeedForward = true;
  
      AutoBuilder.configure(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          swerveDrive::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedForward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  // if this line errors, see note below about kinematics
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
              );
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)
          ),
          config,
          () -> DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
          this
      );
  
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public Command getAutonomousCommand(String pathName) {
    // TODO Auto-generated method stub


    return new PathPlannerAuto(pathName);

  }

//   public void resetOdometry(Pose2d pose) {
//     swerveDrive.resetOdometry(pose);
//   }

//   // Returns the first seen tag ID (if any)
//   public OptionalInt getSeenAprilTagId() {
//     var est = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
//     if (est == null || est.tagCount <= 0 || est.rawFiducials == null || est.rawFiducials.length == 0) {
//       return OptionalInt.empty();
//     }
//     return OptionalInt.of((int) est.rawFiducials[0].id);
//   }

//   public boolean seesTag(int id) {
//     if (!LimelightHelpers.getTV("limelight")) return false;
//     return ((int) LimelightHelpers.getFiducialID("limelight")) == id;
//   }


//   // Pathfind to a target pose (field coordinates)
  public Command goToPose(Pose2d target) {
    PathConstraints constraints = new PathConstraints(
        0.25, 0.25,          // max vel/accel (m/s, m/s^2)
        Math.PI, 2*Math.PI // max ang vel/accel (rad/s, rad/s^2)
    );
    return AutoBuilder.pathfindToPoseFlipped(target, constraints, 0);
  }

 


// public Command alignToHub() {
//   return Commands.run(() -> {
//     if (LimelightHelpers.getTV("limelight")) {
//       // Tag-relative: ty = distance, tx = angle
//       double forwardSpeed = MathUtil.clamp(-LimelightHelpers.getTY("limelight") * 0.05, -0.4, 0.4);
//       double strafeSpeed = 0;
//       double rotSpeed = MathUtil.clamp(-LimelightHelpers.getTX("limelight") * 0.03, -0.5, 0.5);
//       swerveDrive.setChassisSpeeds(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed));
//       System.out.println("tx:" + LimelightHelpers.getTX("limelight") + " ty:" + LimelightHelpers.getTY("limelight"));
//     }
//   }, this).withTimeout(5.0);  // Safety stop
// }

public Command alignToOffset(double offsetX, double offsetY, double headingDeg) {
  return Commands.runOnce(() -> {
    // Verify Tag 10 visible
    if (LimelightHelpers.getFiducialID("limelight") == 10 && LimelightHelpers.getTV("limelight")) {
      // FIXED Tag 10 field location
      Optional<Pose3d> tag10Fixed = fieldLayout.getTagPose(10);
      if (tag10Fixed.isPresent()) {
        Pose2d fixedTag = tag10Fixed.get().toPose2d();
        // YOUR OFFSET from FIXED tag location
        Pose2d target = fixedTag.plus(new Transform2d(offsetX, offsetY, Rotation2d.fromDegrees(headingDeg)));
        swerveDrive.resetOdometry(target);  // Snap to exact spot
        System.out.println("Limelight-verified FIXED offset: " + target);
      }
    }
  }, this);
}

}