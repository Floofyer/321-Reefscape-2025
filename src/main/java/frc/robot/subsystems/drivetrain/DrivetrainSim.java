/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ReefAlign;
import frc.robot.util.SelfControlledSwerveDriveSimulationWrapper;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/*
 * Maplesim drivetrain
 */
@Logged
public class DrivetrainSim implements SwerveDrive {
  private final SelfControlledSwerveDriveSimulationWrapper simulatedDrive;
  private final Field2d field2d;
  private AlignmentSetpoint alignmentSetpoint = new AlignmentSetpoint(Pose2d.kZero, true);
  final DriveTrainSimulationConfig simConfig;
  PIDController headingController;

  private final SwerveDrivePoseEstimator reefPoseEstimator;

  private final Pose2d kRedTopAutoStart = new Pose2d(10.2, 2.2, Rotation2d.kZero);
  private final Pose2d kRedBotAutoStart = new Pose2d(10.2, 5.8, Rotation2d.kZero);

  public DrivetrainSim() {
    this.simConfig =
        DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                COTS.ofMark4(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1),
                    COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                    3)) // L3 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                DrivetrainConstants.kWheelBase, DrivetrainConstants.kTrackWidth)
            // Configures the bumper size (dimensions of the robot bumper) trackwidth + 6 inches
            .withBumperSize(
                DrivetrainConstants.kWheelBase.plus(Inches.of(6)),
                DrivetrainConstants.kTrackWidth.plus(Inches.of(6)))
            .withRobotMass(Pounds.of(113));

    this.simulatedDrive =
        new SelfControlledSwerveDriveSimulationWrapper(
            new SwerveDriveSimulation(simConfig, kRedBotAutoStart));

    this.headingController =
        new PIDController(
            DrivetrainConstants.kTuneHeadingGains.kP(),
            DrivetrainConstants.kTuneHeadingGains.kI(),
            DrivetrainConstants.kTuneHeadingGains.kD());

    this.headingController.enableContinuousInput(-Math.PI, Math.PI);

    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

    // A field2d widget for debugging
    field2d = new Field2d();
    SmartDashboard.putData("Drivetrain Pose Field", field2d);

    this.reefPoseEstimator =
        new SwerveDrivePoseEstimator(
            simulatedDrive.getKinematics(), getHeading(), getModulePositions(), getPose());

    configureAutoBuilder();
    configurePoseControllers();
  }

  @Override
  public Command teleopDrive(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          ChassisSpeeds speeds =
              flipFieldSpeeds(
                  new ChassisSpeeds(
                      translationX.getAsDouble(),
                      translationY.getAsDouble(),
                      rotation.getAsDouble()));
          simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, false);
        });
  }

  @Override
  public Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY) {
    return run(
        () -> {
          Rotation2d desiredRotation =
              flipRotation(
                  Rotation2d.fromRadians(
                      Math.atan2(rotationX.getAsDouble(), rotationY.getAsDouble())));

          ChassisSpeeds speeds =
              flipFieldSpeeds(
                  new ChassisSpeeds(
                      translationX.getAsDouble(),
                      translationY.getAsDouble(),
                      headingController.calculate(
                          getPose().getRotation().getRadians(), desiredRotation.getRadians())));

          simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, false);
        });
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {

    return run(
        () ->
            simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(
                    translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble()),
                new Translation2d(),
                true,
                false));
  }

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () ->
            driveRobotCentric(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                rotation.getAsDouble(),
                DriveFeedforwards.zeros(4)));
  }

  @Override
  public void driveRobotCentric(
      double translationX, double translationY, double rotation, DriveFeedforwards feedforwards) {
    simulatedDrive.runChassisSpeeds(
        new ChassisSpeeds(translationX, translationY, rotation), new Translation2d(), false, false);
  }

  @Override
  public void driveToFieldPose(Pose2d pose) {
    if (pose == null) return;

    ChassisSpeeds targetSpeeds =
        DriverStation.isAutonomous()
            ? new ChassisSpeeds(
                xPoseController.calculate(getPose().getX(), pose.getX())
                    + xPoseController.getSetpoint().velocity,
                yPoseController.calculate(getPose().getY(), pose.getY())
                    + yPoseController.getSetpoint().velocity,
                thetaController.calculate(
                        getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                    + thetaController.getSetpoint().velocity)
            : new ChassisSpeeds(
                xPoseController.calculate(getPose().getX(), pose.getX()),
                yPoseController.calculate(getPose().getY(), pose.getY()),
                thetaController.calculate(
                    getPose().getRotation().getRadians(), pose.getRotation().getRadians()));

    final Pose2d currentPose = getPose();

    if (currentPose.getTranslation().getDistance(alignmentSetpoint.pose().getTranslation())
        < DrivetrainConstants.kAlignmentSetpointTranslationTolerance.in(Meters))
      targetSpeeds = new ChassisSpeeds(0, 0, targetSpeeds.omegaRadiansPerSecond);

    if (Math.abs(
            currentPose.getRotation().minus(alignmentSetpoint.pose().getRotation()).getDegrees())
        < DrivetrainConstants.kAlignmentSetpointRotationTolerance.in(Degrees))
      targetSpeeds =
          new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, 0);

    simulatedDrive.runChassisSpeeds(targetSpeeds, Translation2d.kZero, true, false);
  }

  @Override
  public void driveFixedHeading(double translationX, double translationY, Rotation2d rotation) {
    ChassisSpeeds speeds =
        flipFieldSpeeds(
            new ChassisSpeeds(
                translationX,
                translationY,
                headingController.calculate(
                    getPose().getRotation().getRadians(), rotation.getRadians())));

    simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, false);
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulatedDrive.resetOdometry(pose);
  }

  @Override
  public void setAlignmentSetpoint(AlignmentSetpoint setpoint) {
    alignmentSetpoint = setpoint;
  }

  @Override
  public boolean atPoseSetpoint(Distance tranTol, Angle rotTol) {
    final var currentPose = getPose();
    return currentPose.getTranslation().getDistance(alignmentSetpoint.pose().getTranslation())
            < tranTol.in(Meters)
        && Math.abs(
                currentPose
                    .getRotation()
                    .minus(alignmentSetpoint.pose().getRotation())
                    .getDegrees())
            < rotTol.in(Degrees);
  }

  @Override
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    simulatedDrive.runSwerveStates(states);
  }

  @Logged(name = "MeasuredModuleStates")
  @Override
  public SwerveModuleState[] getMeasuredModuleStates() {
    return simulatedDrive.getMeasuredStates();
  }

  @Logged(name = "MeasuredModulePositions")
  @Override
  public SwerveModulePosition[] getModulePositions() {
    return simulatedDrive.getLatestModulePositions();
  }

  @Logged(name = "TargetModuleStates")
  @Override
  public SwerveModuleState[] getTargetModuleStates() {
    return simulatedDrive.getSetPointsOptimized();
  }

  @Logged(name = "ReefVisionEstimatedPose")
  @Override
  public Pose2d getReefVisionPose() {
    return reefPoseEstimator.getEstimatedPosition();
  }

  @Logged(name = "MeasuredRobotPose")
  @Override
  public Pose2d getPose() {
    return simulatedDrive.getOdometryEstimatedPose();
  }

  @Logged(name = "ActualRobotPose")
  public Pose2d getActualPose() {
    return simulatedDrive.getActualPoseInSimulationWorld();
  }

  @Logged(name = "MeasuredRobotRelativeChassisSpeeds")
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return simulatedDrive.getMeasuredSpeedsRobotRelative(false);
  }

  @Logged(name = "MeasuredHeading")
  @Override
  public Rotation2d getHeading() {
    return simulatedDrive.getDriveTrainSimulation().getGyroSimulation().getGyroReading();
  }

  @Override
  public AlignmentSetpoint getAlignmentSetpoint() {
    return alignmentSetpoint;
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds, standardDeviations);
  }

  @Override
  public void addReefVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations) {
    reefPoseEstimator.addVisionMeasurement(visionRobotPose, timeStampSeconds, standardDeviations);
  }

  @Override
  public void periodic() {
    // update simulated drive and arena
    SimulatedArena.getInstance().simulationPeriodic();
    simulatedDrive.periodic();

    reefPoseEstimator.update(getHeading(), getModulePositions());

    // send simulation data to dashboard for testing
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("odometry").setPose(getPose());
  }

  @Logged(name = "RobotLeftAligned")
  public Pose2d robotLeftAligned() {
    return ReefAlign.leftAlignPoses.get(ReefAlign.getNearestReefID(getPose()));
  }

  @Logged(name = "RobotRightAligned")
  public Pose2d robotRightAligned() {
    return ReefAlign.rightAlignPoses.get(ReefAlign.getNearestReefID(getPose()));
  }

  @Logged(name = "ClosestAprilTag")
  public Pose2d closestAprilTag() {
    return ReefAlign.getNearestReefPose(getPose());
  }
}
