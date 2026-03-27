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
        SmartDashboard.putData("Field", field2d);

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
    PoseEstimate llEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLL);

    if (llEst != null && llEst.tagCount > 0) {
      swerveDrive.addVisionMeasurement(llEst.pose, llEst.timestampSeconds);
    }

    field2d.setRobotPose(swerveDrive.getPose());
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    // TODO Auto-generated method stub

    return swerveDrive;
  }

  public Field2d getField() {
    return field2d;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    driveFieldOrientedWithGyro(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> driveFieldOrientedWithGyro(velocity.get()));
  }

  public void resetGyro() {
    swerveDrive.zeroGyro();
  }

  private void driveFieldOrientedWithGyro(ChassisSpeeds fieldRelativeSpeeds) {
    swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, swerveDrive.getYaw()));
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
  // public Command goToPose(Pose2d target) {
  //   PathConstraints constraints = new PathConstraints(
  //       0.25, 0.25,          // max vel/accel (m/s, m/s^2)
  //       Math.PI, 2*Math.PI // max ang vel/accel (rad/s, rad/s^2)
  //   );
  //   return AutoBuilder.pathfindToPoseFlipped(target, constraints, 0);
  // }

 


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

public boolean seesTag(int tagId){
  int currentTag = (int) LimelightHelpers.getFiducialID(kLL);
  return currentTag == tagId;
}

public Command alignToOffset(double xOffset, double yOffset, double rotationDegrees, int targetTagId) {
  return Commands.defer(() -> {
    // Get the robot's pose in target space (relative to the AprilTag)
    PoseEstimate llEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLL);
   
    if (llEst == null || llEst.tagCount == 0) {
      System.out.println("No AprilTag visible - cannot align");
      return Commands.none();
    }
   
    // Get the current tag ID being tracked
    int tagId = (int) LimelightHelpers.getFiducialID(kLL);

    if (tagId != targetTagId){
      System.out.println("Wrong Tag! Seeing tag " + tagId + ", want tag " + targetTagId);
      return Commands.none();
    }
   
    if (tagId < 0) {
      System.out.println("Invalid tag ID");
      return Commands.none();
    }
   
    // Get the tag's field pose
    Optional<Pose2d> tagPoseOpt = getTagPose(tagId);
   
    if (tagPoseOpt.isEmpty()) {
      System.out.println("Could not find tag " + tagId + " in field layout");
      return Commands.none();
    }
   
    Pose2d tagPose = tagPoseOpt.get();

    //System.out.println("Tag " + targetTagId + " orientation: " + tagPose.getRotation().getDegrees());
   
    // Create the offset transform relative to the tag
    // The tag's rotation determines the coordinate frame
    Transform2d offset = new Transform2d(
      new Translation2d(xOffset, yOffset),
      Rotation2d.fromDegrees(rotationDegrees).minus(tagPose.getRotation())
    );
   
    // Calculate target pose in field coordinates
    Pose2d targetPose = tagPose.plus(offset);
   
    System.out.println("Tag " + tagId + " at " + tagPose);
    System.out.println("Driving to offset: " + targetPose);
   
    // Use PathPlanner's pathfinding to get there
    PathConstraints constraints = new PathConstraints(
      0.25, // max velocity m/s
      0.25, // max acceleration m/s^2
      Units.degreesToRadians(180), // max angular velocity rad/s
      Units.degreesToRadians(360)  // max angular acceleration rad/s^2
    );
   
    // Command pathfindCommand = AutoBuilder.pathfindToPose(
    //   targetPose,
    //   constraints,
    //   0.0 // goal end velocity
    // );

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
   
    //pathfindCommand.schedule();
   
  }, Set.of(this));
  // .until(() -> {
  //   // Stop when we're close enough
  //   PoseEstimate llEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLL);
  //   if (llEst == null || llEst.tagCount == 0) return true;
   
  //   int tagId = (int) LimelightHelpers.getFiducialID(kLL);
  //   Optional<Pose2d> tagPoseOpt = getTagPose(tagId);
   
  //   if (tagPoseOpt.isEmpty()) return true;
   
  //   Pose2d tagPose = tagPoseOpt.get();
  //   Transform2d offset = new Transform2d(
  //     new Translation2d(xOffset, yOffset),
  //     Rotation2d.fromDegrees(rotationDegrees).minus(tagPose.getRotation())
  //   );
  //   Pose2d targetPose = tagPose.plus(offset);
   
  //   double distance = swerveDrive.getPose().getTranslation().getDistance(targetPose.getTranslation());
  //   double angleDiff = Math.abs(swerveDrive.getPose().getRotation().minus(targetPose.getRotation()).getDegrees());
   
  //   return distance < 0.1 && angleDiff < 5; // within 10cm and 5 degrees
  // });
}

/**
 * Helper method to get a tag's pose from the field layout
 */
private Optional<Pose2d> getTagPose(int tagId) {
  Optional<AprilTag> tag = fieldLayout.getTagPose(tagId)
    .map(pose3d -> new AprilTag(tagId, pose3d));
 
  if (tag.isPresent()) {
    return Optional.of(tag.get().pose.toPose2d());
  }
 
  return Optional.empty();
}

}
