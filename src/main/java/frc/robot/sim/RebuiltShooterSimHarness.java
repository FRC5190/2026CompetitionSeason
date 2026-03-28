package frc.robot.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.ironmaple.utils.FieldMirroringUtils;

public final class RebuiltShooterSimHarness {
  private static final Translation3d kBlueHubTarget = new Translation3d(4.5974, 4.034536, 1.5748);
  private static final Translation3d kHubTolerance =
      new Translation3d(RebuiltHub.GoalRadius, RebuiltHub.GoalRadius, RebuiltHub.GoalRadius);

  private final ShooterSimDashboard dashboard_ = new ShooterSimDashboard();

  private SelfControlledSwerveDriveSimulation simulatedDrive_;
  private IntakeSimulation intakeSimulation_;
  private SimulatedArena arena_;

  private GamePieceProjectile activeProjectile_;
  private ActiveShot activeShot_;
  private final Queue<PendingSweepShot> pendingSweepShots_ = new ArrayDeque<>();
  private final List<ShotSampleResult> completedSweepSamples_ = new ArrayList<>();
  private ShotSweepConfig activeSweepConfig_;

  private Optional<ShotSampleResult> lastShot_ = Optional.empty();
  private Optional<ShotSweepResult> lastSweep_ = Optional.empty();
  private Pose3d[] successfulTrajectory_ = new Pose3d[0];
  private Pose3d[] missedTrajectory_ = new Pose3d[0];
  private String lastAction_ = "Ready";

  public void simulationInit() {
    arena_ = SimulatedArena.getInstance();

    DriveTrainSimulationConfig driveTrainConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(edu.wpi.first.units.Units.Kilograms.of(35.0))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(createModuleConfig())
            .withTrackLengthTrackWidth(Inches.of(24.5), Inches.of(24.5))
            .withBumperSize(Meters.of(0.9), Meters.of(0.9));

    simulatedDrive_ =
        new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(driveTrainConfig, new Pose2d(3.0, 4.0, Rotation2d.kZero)))
            .withCurrentLimits(Amps.of(40), Amps.of(20));
    arena_.addDriveTrainSimulation(simulatedDrive_.getDriveTrainSimulation());

    intakeSimulation_ =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            simulatedDrive_.getDriveTrainSimulation(),
            Meters.of(0.6),
            Meters.of(0.25),
            IntakeSimulation.IntakeSide.FRONT,
            20);

    dashboard_.publishDefaults();
    resetFieldAndState();
  }

  public void simulationPeriodic() {
    if (arena_ == null || simulatedDrive_ == null || intakeSimulation_ == null) {
      return;
    }

    ShooterSimDashboard.Inputs inputs = dashboard_.readInputs();

    if (inputs.resetField()) {
      resetFieldAndState();
    }

    if (inputs.clearProjectiles()) {
      clearProjectiles();
      lastAction_ = "Cleared projectiles";
    }

    if (inputs.applyPoseNow()) {
      simulatedDrive_.setSimulationWorldPose(inputs.requestedPose());
      lastAction_ = String.format(
          "Teleported robot to %.2f, %.2f, %.1f deg",
          inputs.requestedPose().getX(),
          inputs.requestedPose().getY(),
          inputs.requestedPose().getRotation().getDegrees());
    }

    simulatedDrive_.runChassisSpeeds(inputs.requestedFieldSpeeds(), new Translation2d(), true, true);

    if (inputs.intakeRunning()) {
      intakeSimulation_.startIntake();
    } else {
      intakeSimulation_.stopIntake();
    }

    if (inputs.loadOneFuel()) {
      intakeSimulation_.addGamePieceToIntake();
      lastAction_ = "Added one fuel to intake";
    }

    if (inputs.runSweep()) {
      beginSweep(inputs);
    }

    if (inputs.fireNow()) {
      launchManualShot(inputs);
    }

    if (activeProjectile_ == null && !pendingSweepShots_.isEmpty()) {
      launchPendingSweepShot();
    }

    arena_.simulationPeriodic();
    simulatedDrive_.periodic();
    updateActiveShotResolution();

    Pose2d robotPose = simulatedDrive_.getActualPoseInSimulationWorld();
    Pose3d hubPose = new Pose3d(getCurrentAllianceHubTarget(), new edu.wpi.first.math.geometry.Rotation3d());
    dashboard_.publishRobotPose(robotPose, hubPose);
    dashboard_.publishFuelPoses(arena_.getGamePiecesArrayByType("Fuel"));
    dashboard_.publishTrajectories(successfulTrajectory_, missedTrajectory_);
    dashboard_.publishState(
        robotPose,
        intakeSimulation_.getGamePiecesAmount(),
        fuelOnFieldCount(),
        fuelInAirCount(),
        activeProjectile_ != null,
        activeSweepConfig_ != null,
        completedSweepSamples_.size(),
        activeSweepTotalShots(),
        lastShot_,
        lastSweep_,
        allianceScore(),
        lastAction_);
  }

  private void launchManualShot(ShooterSimDashboard.Inputs inputs) {
    if (activeProjectile_ != null) {
      lastAction_ = "Shot ignored: projectile still active";
      return;
    }

    if (inputs.useIntakeAmmo() && !intakeSimulation_.obtainGamePieceFromIntake()) {
      lastAction_ = "Shot blocked: no fuel in intake";
      return;
    }

    LaunchRequest request =
        new LaunchRequest(
            simulatedDrive_.getActualPoseInSimulationWorld(),
            inputs.requestedFieldSpeeds(),
            inputs.shooterRpm(),
            inputs.hoodDeg(),
            inputs.turretDeg(),
            inputs.launchHeightM(),
            new Translation2d(inputs.shooterOffsetXM(), inputs.shooterOffsetYM()),
            inputs.launchSpeedAt6000RpmMps());
    launchShot(request, false);
    lastAction_ = String.format(
        "Manual shot launched at %.0f rpm and %.1f deg", request.rpm(), request.hoodDeg());
  }

  private void beginSweep(ShooterSimDashboard.Inputs inputs) {
    if (activeProjectile_ != null || activeSweepConfig_ != null) {
      lastAction_ = "Sweep ignored: sweep or shot already active";
      return;
    }

    activeSweepConfig_ =
        new ShotSweepConfig(
                inputs.sweepRpmMin(),
                inputs.sweepRpmMax(),
                inputs.sweepRpmStep(),
                inputs.sweepHoodMinDeg(),
                inputs.sweepHoodMaxDeg(),
                inputs.sweepHoodStepDeg(),
                inputs.turretDeg(),
                inputs.launchHeightM(),
                inputs.shooterOffsetXM(),
                inputs.shooterOffsetYM(),
                inputs.launchSpeedAt6000RpmMps(),
                inputs.requestedPose(),
                inputs.requestedFieldSpeeds())
            .normalize();

    pendingSweepShots_.clear();
    completedSweepSamples_.clear();
    clearTrajectoryPublishers();

    for (double rpm : activeSweepConfig_.rpmValues()) {
      for (double hoodDeg : activeSweepConfig_.hoodValuesDeg()) {
        pendingSweepShots_.add(
            new PendingSweepShot(
                new LaunchRequest(
                    activeSweepConfig_.robotPose(),
                    activeSweepConfig_.chassisSpeedsFieldRelative(),
                    rpm,
                    hoodDeg,
                    activeSweepConfig_.turretDeg(),
                    activeSweepConfig_.launchHeightM(),
                    new Translation2d(
                        activeSweepConfig_.shooterOffsetXM(), activeSweepConfig_.shooterOffsetYM()),
                    activeSweepConfig_.launchSpeedAt6000RpmMps())));
      }
    }

    lastAction_ =
        String.format("Started sweep with %d shots", pendingSweepShots_.size());
  }

  private void launchPendingSweepShot() {
    PendingSweepShot pendingShot = pendingSweepShots_.poll();
    if (pendingShot == null) {
      return;
    }

    simulatedDrive_.setSimulationWorldPose(pendingShot.launchRequest().robotPose());
    launchShot(pendingShot.launchRequest(), true);
  }

  private void launchShot(LaunchRequest request, boolean fromSweep) {
    clearTrajectoryPublishers();

    RebuiltFuelOnFly projectile =
        new RebuiltFuelOnFly(
            request.robotPose().getTranslation(),
            request.shooterOffsetOnRobot(),
            request.chassisSpeedsFieldRelative(),
            request.robotPose().getRotation().plus(Rotation2d.fromDegrees(request.turretDeg())),
            Meters.of(request.launchHeightM()),
            MetersPerSecond.of(request.launchSpeedMps()),
            Degrees.of(request.hoodDeg()));

    projectile
        .withTargetPosition(this::getCurrentAllianceHubTarget)
        .withTargetTolerance(kHubTolerance)
        .withProjectileTrajectoryDisplayCallBack(
            poses -> successfulTrajectory_ = poses.toArray(Pose3d[]::new),
            poses -> missedTrajectory_ = poses.toArray(Pose3d[]::new));

    activeProjectile_ = projectile;
    activeShot_ = new ActiveShot(request, fromSweep, allianceScore(), Timer.getFPGATimestamp());
    arena_.addGamePieceProjectile(projectile);
  }

  private void updateActiveShotResolution() {
    if (activeProjectile_ == null || activeShot_ == null) {
      if (activeSweepConfig_ != null && pendingSweepShots_.isEmpty()) {
        finishSweepIfReady();
      }
      return;
    }

    boolean resolved =
        activeProjectile_.hasHitGround()
            || activeProjectile_.hasGoneOutOfField()
            || allianceScore() > activeShot_.allianceScoreBeforeLaunch();

    if (!resolved) {
      return;
    }

    ShotSampleResult result =
        new ShotSampleResult(
            activeShot_.request().rpm(),
            activeShot_.request().hoodDeg(),
            activeShot_.request().turretDeg(),
            activeShot_.request().launchSpeedMps(),
            activeShot_.request().robotPose(),
            activeShot_.request().chassisSpeedsFieldRelative(),
            allianceScore() > activeShot_.allianceScoreBeforeLaunch(),
            activeShot_.flightTimeSec(),
            activeProjectile_.getPose3d());

    lastShot_ = Optional.of(result);
    if (activeShot_.fromSweep()) {
      completedSweepSamples_.add(result);
    }

    activeProjectile_ = null;
    activeShot_ = null;

    if (activeSweepConfig_ != null && pendingSweepShots_.isEmpty()) {
      finishSweepIfReady();
    }
  }

  private void finishSweepIfReady() {
    if (activeSweepConfig_ == null || completedSweepSamples_.size() < activeSweepTotalShots()) {
      return;
    }

    lastSweep_ =
        Optional.of(new ShotSweepResult(activeSweepConfig_, List.copyOf(completedSweepSamples_)));
    lastAction_ = "Sweep complete";
    activeSweepConfig_ = null;
    completedSweepSamples_.clear();
    pendingSweepShots_.clear();
  }

  private void resetFieldAndState() {
    arena_.resetFieldForAuto();
    intakeSimulation_.setGamePiecesCount(0);
    clearProjectiles();
    lastSweep_ = Optional.empty();
    lastShot_ = Optional.empty();
    lastAction_ = "Field reset";
  }

  private void clearProjectiles() {
    if (arena_ != null && activeProjectile_ != null) {
      arena_.removeProjectile(activeProjectile_);
    }
    activeProjectile_ = null;
    activeShot_ = null;
    pendingSweepShots_.clear();
    activeSweepConfig_ = null;
    completedSweepSamples_.clear();
    clearTrajectoryPublishers();
  }

  private void clearTrajectoryPublishers() {
    successfulTrajectory_ = new Pose3d[0];
    missedTrajectory_ = new Pose3d[0];
    dashboard_.clearTrajectoryPublishers();
  }

  private int activeSweepTotalShots() {
    if (activeSweepConfig_ == null) {
      return 0;
    }
    return activeSweepConfig_.rpmValues().size() * activeSweepConfig_.hoodValuesDeg().size();
  }

  private int fuelOnFieldCount() {
    return (int) arena_.gamePiecesOnField().stream().filter(piece -> "Fuel".equals(piece.getType())).count();
  }

  private int fuelInAirCount() {
    return (int) arena_.gamePieceLaunched().stream().filter(piece -> "Fuel".equals(piece.getType())).count();
  }

  private int allianceScore() {
    return arena_.getScore(currentAllianceIsBlue());
  }

  private boolean currentAllianceIsBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  private Translation3d getCurrentAllianceHubTarget() {
    return FieldMirroringUtils.toCurrentAllianceTranslation(kBlueHubTarget);
  }

  private static SwerveModuleSimulationConfig createModuleConfig() {
    return new SwerveModuleSimulationConfig(
        DCMotor.getNEO(1),
        DCMotor.getNEO(1),
        6.12,
        21.42,
        edu.wpi.first.units.Units.Volts.of(0.1),
        edu.wpi.first.units.Units.Volts.of(0.2),
        Inches.of(2),
        KilogramSquareMeters.of(0.03),
        COTS.WHEELS.COLSONS.cof);
  }

  private record LaunchRequest(
      Pose2d robotPose,
      ChassisSpeeds chassisSpeedsFieldRelative,
      double rpm,
      double hoodDeg,
      double turretDeg,
      double launchHeightM,
      Translation2d shooterOffsetOnRobot,
      double launchSpeedAt6000RpmMps) {

    private double launchSpeedMps() {
      return (rpm / 6000.0) * launchSpeedAt6000RpmMps;
    }
  }

  private record ActiveShot(
      LaunchRequest request,
      boolean fromSweep,
      int allianceScoreBeforeLaunch,
      double launchTimestampSec) {

    private double flightTimeSec() {
      return Math.max(0.0, Timer.getFPGATimestamp() - launchTimestampSec);
    }
  }

  private record PendingSweepShot(LaunchRequest launchRequest) {}
}
