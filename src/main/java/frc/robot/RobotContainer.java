/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutomaticAutonomousMaker3000;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.StationAlign;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReefPosition;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.disable();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.disable();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private Climber climber = Climber.create();

  private CoralSuperstructure coralSuperstructure =
      new CoralSuperstructure(elevator, elevatorArm, coralEndEffector);
  private AlgaeSuperstructure algaeSuperstructure =
      new AlgaeSuperstructure(algaePivot, algaeRollers);

  private AutomaticAutonomousMaker3000 automaker =
      new AutomaticAutonomousMaker3000(drivetrain, coralSuperstructure);

  private Vision vision =
      Vision.create(
          // Java 21 pattern matching switch would be nice
          (drivetrain instanceof DrivetrainSim)
              ? ((DrivetrainSim) drivetrain)::getActualPose
              : drivetrain::getPose,
          visionEst ->
              drivetrain.addVisionMeasurement(
                  visionEst.estimate().estimatedPose.toPose2d(),
                  visionEst.estimate().timestampSeconds,
                  visionEst.stdDevs()),
          reefVisionEst ->
              drivetrain.addReefVisionMeasurement(
                  reefVisionEst.estimate().estimatedPose.toPose2d(),
                  reefVisionEst.estimate().timestampSeconds,
                  reefVisionEst.stdDevs()));

  private CommandXboxController driver = new CommandXboxController(0);
  private XboxController manipulator = new XboxController(1);

  private boolean isDriverOverride = false;
  private boolean isClimbing = false;

  private Trigger isSlowMode =
      driver
          .leftBumper()
          .or(() -> elevator.getHeight().in(Meters) > ElevatorConstants.kSlowedHeight.in(Meters))
          .or(
              () ->
                  isClimbing
                      && ClimberConstants.kBargeZone.contains(
                          drivetrain.getPose().getTranslation()));

  private DoubleSupplier driverForward =
      () ->
          -MathUtil.applyDeadband(
                  Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                  DrivetrainConstants.kDriveDeadband)
              * Math.cos(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * (isSlowMode.getAsBoolean()
                  ? 1.5
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));

  private DoubleSupplier driverStrafe =
      () ->
          -MathUtil.applyDeadband(
                  Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                  DrivetrainConstants.kDriveDeadband)
              * Math.sin(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * (isSlowMode.getAsBoolean()
                  ? 1.5
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));

  private DoubleSupplier driverTurn =
      () ->
          -MathUtil.applyDeadband(driver.getRightX(), DrivetrainConstants.kRotationDeadband)
              * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond);

  // robot queued states
  private ReefPosition queuedReefPosition = ReefPosition.RIGHT;
  private CoralScorerSetpoint queuedSetpoint = CoralScorerSetpoint.L4;

  private SuperstructureVisualizer stateVisualizer =
      new SuperstructureVisualizer(
          () -> elevator.getHeight(), () -> elevatorArm.getAngle(), () -> climber.getAngle());

  private Leds leds = Leds.getInstance();
  private AddressableLEDSim ledSim = new AddressableLEDSim(leds.strip);

  private Trigger isAlgaeSetpoint =
      new Trigger(
          () ->
              queuedSetpoint == CoralScorerSetpoint.ALGAE_LOW
                  || queuedSetpoint == CoralScorerSetpoint.ALGAE_HIGH);
  private Trigger isCoralSetpoint =
      new Trigger(
          () ->
              queuedSetpoint == CoralScorerSetpoint.L1
                  || queuedSetpoint == CoralScorerSetpoint.L2
                  || queuedSetpoint == CoralScorerSetpoint.L3
                  || queuedSetpoint == CoralScorerSetpoint.L4);

  private DoubleSupplier reefAlignProgressPercent =
      () ->
          leds.calculateProgressBar(
              elevator.getHeight(),
              coralSuperstructure.getTargetHeight(),
              elevatorArm.getAngle(),
              coralSuperstructure.getTargetAngle(),
              drivetrain.getPose(),
              drivetrain.getAlignmentSetpoint().pose());

  public RobotContainer() {

    // reset elevator arm encoder on robot enable
    RobotModeTriggers.disabled()
        .negate()
        .onTrue(elevatorArm.seedEncoder().onlyIf(() -> !elevatorArm.hasSeeded()));

    // home everything on robot start
    RobotModeTriggers.disabled()
        .negate()
        .and(RobotModeTriggers.teleop())
        .onTrue(elevator.homeEncoder().onlyIf(() -> !elevator.elevatorIsHomed()));

    // RobotModeTriggers.disabled().negate().onTrue(HomingCommands.homeClimber(climber));

    // drive
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));

    // full-featured default commnds
    // algaeRollers.setDefaultCommand(algaeRollers.stallIfHasAlgae());
    // algaePivot.setDefaultCommand(algaePivot.goToAngle(() ->
    // AlgaeSetpoint.NEUTRAL.getAlgaeAngle()));
    elevator.setDefaultCommand(
        elevator.goToHeight(() -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight()));
    elevatorArm.setDefaultCommand(
        elevatorArm.goToAngleProfiled(() -> CoralScorerSetpoint.NEUTRAL.getArmAngle()));

    coralEndEffector.setDefaultCommand(coralEndEffector.stallCoralIfDetected());

    // testing default commands
    algaeRollers.setDefaultCommand(algaeRollers.setMechanismVoltage(() -> Volts.zero()));
    algaePivot.setDefaultCommand(algaePivot.setMechanismVoltage(() -> Volts.zero()));
    // elevator.setDefaultCommand(elevator.setVoltage(() -> Volts.zero()));
    // elevatorArm.setDefaultCommand(elevatorArm.runVolts(() -> Volts.zero()));
    // coralEndEffector.setDefaultCommand(coralEndEffector.runVolts(() -> Volts.of(1)));

    climber.setDefaultCommand(
        // climber.setMechanismVoltage(() -> Volts.zero())
        climber.goToAngle(() -> ClimberConstants.kDefaultAngle));

    leds.setDefaultCommand(leds.updateLeds());

    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    // new Trigger(algaePivot::inCollisionZone)
    //     .and(new Trigger(elevator::inCollisionZone))
    //     .onTrue(
    //         elevator
    //             .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
    //             .until(new Trigger(algaePivot::inCollisionZone).negate()));

    configureLeds();
    configManipTriggers();
    configureBindings();
    // configureTuningBindings();
  }

  private double volts = 0;

  private void configureTuningBindings() {
    // driver.a().whileTrue(elevatorArm.tune());
    // driver.y().whileTrue(coralEndEffector.tune());
    driver.a().whileTrue(coralSuperstructure.tune());
    // driver.b().whileTrue(coralSuperstructure.feedCoral());
    driver.leftBumper().whileTrue(coralEndEffector.intakeCoral());
    driver.rightBumper().whileTrue(coralEndEffector.outtakeCoral());

    // driver.a().whileTrue(coralEndEffector.runAtVelocity(() -> RPM.of(-2000)));
    // driver.b().whileTrue(coralEndEffector.runAtVelocity(() -> RPM.of(-3000)));
    // driver.x().whileTrue(coralEndEffector.runAtVelocity(() -> RPM.of(-4000)));
    // driver.y().whileTrue(coralEndEffector.runAtVelocity(() -> RPM.of(-5000)));

    // driver.a().whileTrue(climber.tune());

    // climb!
    // driver.y().toggleOnTrue(climber.goToAngle(() -> ClimberConstants.kClimbPrepAngle));

    // driver.a().onTrue(climber.climb());

    // driver.b().toggleOnTrue(coralSuperstructure.goToSetpoint(() -> CoralScorerSetpoint.CLIMB));
    // driver.y().whileTrue(algaePivot.setMechanismVoltage(() -> Volts.of(1)));
    // driver.a().whileTrue(algaePivot.setMechanismVoltage(() -> Volts.of(-1)));
    // driver.x().whileTrue(algaePivot.setMechanismVoltage(() -> Volts.of(volts)));

    // driver
    //     .leftBumper()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               volts += 0.1;
    //               System.out.println("Changing volts to: " + volts);
    //             }));

    // driver
    //     .rightBumper()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               volts -= 0.1;
    //               System.out.println("Changing volts to: " + volts);
    //             }));

    driver.y().whileTrue(ReefAlign.tuneAlignment(drivetrain));

    // driver.b().whileTrue(coralSuperstructure.feedCoral());

    // driver.leftBumper().whileTrue(elevator.setVoltage(() -> Volts.of(1)));
    // driver.rightBumper().whileTrue(elevator.setVoltage(() -> Volts.of(-1)));

    // driver.povLeft().whileTrue(elevatorArm.runVolts(() -> Volts.of(1)));
    // driver.povRight().whileTrue(elevatorArm.runVolts(() -> Volts.of(-1)));

    // tune elevator
    // driver.a().whileTrue(elevator.tune());

    // tune elevator arm
    // driver.a().whileTrue(elevatorArm.tune());

    // find arm setpoints
    // driver.y().whileTrue(coralSuperstructure.tune());
    // driver.leftBumper().whileTrue(coralSuperstructure.feedCoral());
    // driver.rightBumper().whileTrue(coralEndEffector.outtakeCoral());

    // alignment testing (no arm)
    // driver.a().whileTrue(ReefAlign.rotateToNearestReefTag(drivetrain, driverForward,
    // driverStrafe));
    // driver.b().whileTrue(ReefAlign.alignToReef(drivetrain, () -> ReefPosition.RIGHT));

    // test algae intake
    // driver.b().whileTrue(algaeSuperstructure.intakeAlgae());
    // driver.a().whileTrue(algaeSuperstructure.outtakeAlgae());
  }

  private void configureLeds() {
    // Driving LED signals
    leds.registerSignal(0, () -> true, () -> LedsConstants.kDefault);
    leds.registerSignal(2, () -> coralSuperstructure.hasCoral(), () -> LedsConstants.kHasCoral);
    leds.registerSignal(
        3,
        () -> algaeSuperstructure.hasAlgae() && coralSuperstructure.hasCoral(),
        () -> LedsConstants.kHasCoralAndAlgae);
    leds.registerSignal(4, () -> leds.isIntaking, () -> LedsConstants.kIntaking);
    leds.registerSignal(5, () -> leds.isOuttaking, () -> LedsConstants.kOuttaking);
    leds.registerSignal(6, () -> leds.isRotateAligning, () -> LedsConstants.kRotationAligning);

    leds.registerSignal(
        7,
        () ->
            leds.isRotateAligning
                && ReefAlign.isWithinReefRange(drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                && Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) >= 0.05,
        () -> LedsConstants.kReadyToAlign);

    leds.registerSignal(
        8, () -> leds.isReefAligning, () -> LedsConstants.kReefAligning(reefAlignProgressPercent));

    leds.registerSignal(
        9,
        () -> leds.isReefAligning && drivetrain.atFinalPoseSetpoint(),
        () -> LedsConstants.kReefAligned);

    // when we are aligned, also works when manually aligning
    leds.registerSignal(10, () -> isDriverOverride, () -> LedsConstants.kAlignOverride);
    leds.registerSignal(11, () -> isClimbing, () -> LedsConstants.kClimbing);

    // Error State LED Signals
    leds.registerSignal(
        99, () -> !vision.areCamerasConnected(), () -> LedsConstants.kVisionDisconnect);
    // leds.registerSignal(100, () -> !DriverStation.isDSAttached(), () ->
    // LedsConstants.kRobotDisconnect);
  }

  private void configureBindings() {
    // driver controls
    // score coral / flip off algae
    driver
        .y()
        .toggleOnTrue(
            climber
                .goToAngle(() -> ClimberConstants.kClimbPrepAngle)
                .alongWith(
                    coralSuperstructure
                        .goToSetpointPID(() -> CoralScorerSetpoint.CLIMB)
                        .beforeStarting(() -> isClimbing = true)
                        .finallyDo(() -> isClimbing = false)));
    driver
        .a()
        .onTrue(
            climber
                .climb()
                .alongWith(coralSuperstructure.goToSetpointPID(() -> CoralScorerSetpoint.CLIMB)));

    driver.b().onTrue(elevatorArm.seedEncoder());

    // driver.a().whileTrue(elevator.setVoltage(() -> Volts.of(2)));
    // driver.b().whileTrue(elevator.setVoltage(() -> Volts.of(-2)));

    // --- CORAL AUTOMATED CONTROLS ---
    // RIGHT BUMPER + CORAL MODE = INTAKE CORAL
    // RIGHT TRIGGER + CORAL MODE = AUTO ALIGN TO CORAL
    // RIGHT TRIGGER RELEASE + CORAL MODE = OUTTAKE CORAL

    // RIGHT BUMPER + CORAL MODE = INTAKE CORAL
    driver
        .rightBumper()
        .and(isCoralSetpoint)
        .whileTrue(
            StationAlign.rotateToNearestStationTag(drivetrain, driverForward, driverStrafe)
                .onlyWhile(() -> StationAlign.getStationDistance(drivetrain) < 2)
                .andThen(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn))
                .until(() -> StationAlign.getStationDistance(drivetrain) < 2)
                .repeatedly()
                .alongWith(
                    coralSuperstructure
                        .feedCoral()
                        .asProxy()
                        .until(() -> coralEndEffector.hasCoral())
                    // .andThen(
                    //     ControllerCommands.rumbleController(
                    //         driver.getHID(), Seconds.of(0.5), RumbleType.kRightRumble, 0.75)
                    //         )
                    ));

    // RIGHT TRIGGER + CORAL MODE = AUTO ALIGN TO CORAL
    driver
        .rightTrigger()
        .and(isCoralSetpoint)
        .whileTrue(
            Commands.runOnce(() -> isDriverOverride = false)
                .andThen(
                    // either align to reef or coral based on how far we are away rotate to reef
                    // until we're close enough
                    ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe)
                        .until(
                            () ->
                                ReefAlign.isWithinReefRange(
                                        drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                                    // use mechanism threshold cuz we wanna be close before aligning
                                    // in this case
                                    && Math.hypot(
                                            driverForward.getAsDouble(), driverStrafe.getAsDouble())
                                        <= 0.075
                                    && !isDriverOverride)
                        .andThen(
                            // when we get close enough, align to reef, but only while we're
                            // close enough
                            Commands.sequence(
                                    // ReefAlign.alignToPrealignReef(
                                    //         drivetrain, () -> queuedReefPosition)
                                    //     .onlyWhile(
                                    //         () ->
                                    //             coralSuperstructure
                                    //                     .getTargetHeight()
                                    //                     .isEquivalent(
                                    //                         CoralScorerSetpoint.NEUTRAL
                                    //                             .getElevatorHeight())
                                    //                 && coralSuperstructure
                                    //                     .getTargetAngle()
                                    //                     .isEquivalent(
                                    //                         queuedSetpoint.getArmAngle())),
                                    ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition))
                                .onlyWhile(
                                    () ->
                                        ReefAlign.isWithinReefRange(
                                                drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                                            && Math.hypot(
                                                    driverForward.getAsDouble(),
                                                    driverStrafe.getAsDouble())
                                                <= 0.075
                                            &&
                                            // allow driver control to be taken back when
                                            // driverOverride becomes true
                                            !isDriverOverride))
                        // when we get far away, repeat the command
                        .repeatedly())
                .alongWith(
                    coralSuperstructure
                        .goToSetpointPID(
                            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            () -> queuedSetpoint.getArmAngle())
                        .until(
                            () ->
                                ReefAlign.isWithinReefRange(
                                        drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                                    && coralSuperstructure.atTargetState(
                                        CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                                        queuedSetpoint.getArmAngle()))
                        .andThen(
                            coralSuperstructure
                                .goToSetpointProfiled(
                                    () ->
                                        Meters.of(
                                            Math.min(
                                                queuedSetpoint.getElevatorHeight().in(Meters),
                                                CoralScorerSetpoint.PREALIGN
                                                    .getElevatorHeight()
                                                    .in(Meters))),
                                    () -> queuedSetpoint.getArmAngle())
                                .until(drivetrain::atFinalPoseSetpoint)
                                .andThen(
                                    coralSuperstructure.goToSetpointProfiled(() -> queuedSetpoint))
                                .onlyWhile(
                                    () ->
                                        ReefAlign.isWithinReefRange(
                                            drivetrain, ReefAlign.kMechanismDeadbandThreshold)))
                        .repeatedly()));

    // RIGHT TRIGGER RELEASE + CORAL MODE = OUTTAKE CORAL
    driver
        .rightTrigger()
        .and(isCoralSetpoint)
        .onFalse( // for coral scoring
            coralSuperstructure
                .goToSetpointPID(() -> queuedSetpoint) // ensure we're at the setpoint
                .alongWith(coralSuperstructure.outtakeCoral())
                .onlyIf(
                    () ->
                        coralSuperstructure.atTargetState(queuedSetpoint)
                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL
                            && !driver.povLeft().getAsBoolean()
                            && ReefAlign.isWithinReefRange(
                                drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                            && isCoralSetpoint.getAsBoolean()) // and outtake coral
                .withTimeout(0.5) // timeout at 1 second
                .andThen(
                    // move arm up and go back down (only if we're already at the scoring setpoint
                    // state)
                    coralSuperstructure
                        .goToSetpointPID(
                            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            () -> CoralScorerSetpoint.PREALIGN.getArmAngle())
                        .until(
                            () ->
                                coralSuperstructure
                                        .getElevator()
                                        .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight())
                                    && !ReefAlign.isWithinReefRange(
                                        drivetrain, ReefAlign.kMechanismDeadbandThreshold))
                    // .onlyIf(
                    //     () ->
                    //         !coralSuperstructure
                    //             .getElevator()
                    //             .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight()))
                    ));

    // --- CORAL MANUAL CONTROLS ---
    // LEFT TRIGGER + CORAL MODE = ALIGN CORAL MANUALLY

    driver
        .leftTrigger()
        .and(isCoralSetpoint)
        .whileTrue(coralSuperstructure.goToSetpointPID(() -> queuedSetpoint));

    // LEFT TRIGGER RELEASE + CORAL MODE = OUTTAKE CORAL
    driver
        .leftTrigger()
        .onFalse(
            coralSuperstructure
                .goToSetpointPID(() -> queuedSetpoint) // ensure we're at the setpoint
                .alongWith(coralSuperstructure.outtakeCoral())
                .onlyIf(
                    () ->
                        coralSuperstructure.atTargetState(queuedSetpoint)
                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL
                            && !driver.povLeft().getAsBoolean()
                            && isCoralSetpoint.getAsBoolean()) // and outtake coral
                .withTimeout(0.5)
                .andThen(
                    // move arm up and go back down (only if we're already at the scoring setpoint
                    // state)
                    coralSuperstructure
                        .goToSetpointPID(
                            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            () -> CoralScorerSetpoint.PREALIGN.getArmAngle())
                        .until(
                            () ->
                                coralSuperstructure
                                        .getElevator()
                                        .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight())
                                    && !ReefAlign.isWithinReefRange(
                                        drivetrain,
                                        ReefAlign
                                            .kMechanismDeadbandThreshold))) // timeout at 1 second
            );

    // toggle driver override
    driver.povUp().onTrue(Commands.runOnce(() -> isDriverOverride = !isDriverOverride));

    // --- ALGAE AUTOMATED-ISH CONTROLS ---
    // RIGHT BUMPER + ALGAE MODE = INTAKE ALGAE
    // RIGHT TRIGGER + ALGAE MODE (PROCESSOR) = GO TO PROCESSOR POSITION + ALIGN (not implemented)
    // RIGHT TRIGGER RELEASE + ALGAE MODE (PROCESSOR) = OUTTAKE ALGAE (not implemented)
    // RIGHT TRIGGER + ALGAE MODE (BARGE) = GO TO BARGE POSITION
    // RIGHT TRIGGER RELEASE + ALGAE MODE (BARGE) = BARGE RELEASE

    // RIGHT BUMPER + ALGAE MODE = INTAKE ALGAE
    driver
        .rightTrigger()
        .and(isAlgaeSetpoint)
        .and(() -> !coralSuperstructure.hasCoral())
        .whileTrue(
            coralSuperstructure
                .goToSetpointPID(() -> queuedSetpoint)
                .alongWith(
                    coralSuperstructure.knockAlgae(),
                    ReefAlign.rotateToNearestReefTagFullField(
                        drivetrain, driverForward, driverStrafe)));
  }

  public void configManipTriggers() {

    // manip controls
    // 1 to 4 - right side L1-L4
    // 5 to 8 - left side L1-L4
    // 9 to 10 - algae low / high

    manipTrigger(2)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(7)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    manipTrigger(1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(6)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    new Trigger(() -> manipulator.getPOV() == 270)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_LOW;
                }));

    new Trigger(() -> manipulator.getPOV() == 180)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_HIGH;
                }));
  }

  private Trigger manipTrigger(int button) {
    return new Trigger(() -> manipulator.getRawButton(button));
  }

  public Command getAutonomousCommand() {
    return automaker.getStoredAuto();
  }
}
