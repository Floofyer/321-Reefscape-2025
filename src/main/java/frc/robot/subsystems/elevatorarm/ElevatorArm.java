/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.util.TunableConstant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// NOTE: Convention is: zeroed when coral intake CG is 90 degrees
// TO ZERO: move coral intake such that CG is all the way down due to gravity, zero, then move it to
// 90 deg, then
// zero again

// Elevator Arm subsystem - represents the arm/pivot on the elevator
@Logged
public class ElevatorArm extends SubsystemBase {
  // hardware abstraction for the arm
  private ElevatorArmIO io;
  private ElevatorArmInputs inputs;

  // controllers classes for controlling the arm
  private PIDController pidController;
  private ArmFeedforward feedforward;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private double goalAngle;
  private boolean hasSeeded = false;

  // config for the arm
  private ElevatorArmConfig config;

  // suppliers for game piece detection for variable feedforward for game pieces
  @NotLogged private BooleanSupplier hasCoral = () -> false;

  /**
   * Creates an ElevatorArm instance controlling a real or simulated elevator arm based on the
   * environment
   */
  public static ElevatorArm create() {
    return RobotBase.isReal()
        ? new ElevatorArm(new ElevatorArmIOSpark(), ElevatorArmIOSpark.config)
        : new ElevatorArm(new ElevatorArmIOSim(), ElevatorArmIOSim.config);
  }

  /** Creates an ElevatorArm that does not control any hardware / simulation (to disable the arm) */
  public static ElevatorArm disable() {
    return new ElevatorArm(new ElevatorArmIOIdeal(), ElevatorArmIOIdeal.config);
  }

  /**
   * Instantiates an ElevatorArm
   *
   * @param io the hardware abstraction interface that the ElevatorArm is controlling
   * @param config the tuning config for the ElevatorArm
   */
  public ElevatorArm(ElevatorArmIO io, ElevatorArmConfig config) {
    this.io = io;
    this.config = config;
    this.inputs = new ElevatorArmInputs();
    this.pidController = new PIDController(config.kP(), config.kI(), config.kD());
    this.feedforward = new ArmFeedforward(0, config.kG(), 0);

    this.pidController.setTolerance(ElevatorArmConstants.kAngleTolerance.in(Degrees));
    this.profile = new TrapezoidProfile(ElevatorArmConstants.kArmConstraints);
  }

  /**
   * Calculates the amount of feedforward needed to keep the arm with the game piece up
   *
   * @return the amount of feedforward (in volts) to keep the arm with game piece up
   */
  private double calculateGamepieceFeedforward(Angle targetAngle) {
    // calculate the amount of feedforward needed to keep a coral
    double output = 0;
    if (hasCoral.getAsBoolean()) {
      output += config.kCoralFF() * Math.cos(targetAngle.in(Radians));
    }
    return output;
  }

  /**
   * Commands the arm to go to a desired angle This needs to be run continuously (see {@link
   * ElevatorArm#goToAngle(Supplier<Angle>) goToAngle})
   *
   * @param angle the angle to command the arm to go to
   */
  public void goToAngle(Angle angle) {
    setpointState =
        profile.calculate(
            RobotConstants.kRobotLoopPeriod.in(Seconds),
            setpointState,
            new TrapezoidProfile.State(angle.in(Degrees), 0));

    double volts =
        pidController.calculate(inputs.angle.in(Degrees), setpointState.position)
            + feedforward.calculate(
                Degrees.of(setpointState.position).plus(ElevatorArmConstants.kCMOffset).in(Radians),
                0)
            + calculateGamepieceFeedforward(Degrees.of(setpointState.position));

    io.setVoltage(Volts.of(volts));
  }

  /**
   * Commands the arm to run at a certain voltage This needs to be run continuously (see {@link
   * ElevatorArm#runVolts(Supplier<Voltage>) runVolts})
   *
   * @param volts the voltage to run the arm at
   */
  public void runVolts(Voltage volts) {
    io.setVoltage(volts);
  }

  /**
   * Creates a command that runs the arm to a certain angle NOTE: this command NEVER ends
   *
   * @param angleSup A supplier that supplies the angle for the arm to go to
   * @return a command that runs the arm to the desired angle supplied by the Supplier<Angle>
   */
  public Command goToAngleProfiled(Supplier<Angle> angleSup) {
    return run(() -> {
          goalAngle = angleSup.get().in(Degrees);
          goToAngle(angleSup.get());
        })
        .beforeStarting(() -> profile = new TrapezoidProfile(ElevatorArmConstants.kArmConstraints));
  }

  public Command goToAnglePID(Supplier<Angle> angleSup) {
    return run(() -> {
          goalAngle = angleSup.get().in(Degrees);
          goToAngle(angleSup.get());
        })
        .beforeStarting(() -> profile = new TrapezoidProfile(new Constraints(0, 0)));
  }

  /**
   * Creates a command that runs the arm at a certain voltage
   *
   * @param volts A supplier that gives the voltage for the arm to run at
   * @return a command that runs the arm at the voltage suppplied by the Supplier<Voltage>
   */
  public Command runVolts(Supplier<Voltage> volts) {
    return run(
        () -> {
          runVolts(volts.get());
        });
  }

  public Command seedEncoder() {
    return Commands.runOnce(
        () -> {
          io.seedEncoderValues();
          System.out.println("Seeded.");
          this.hasSeeded = true;
        });
  }

  public boolean hasSeeded() {
    return hasSeeded;
  }

  /**
   * Creates a Command that allows the user to tune the ElevatorArm using SmartDashboard
   *
   * <p>Parameters: kP, kI, kD, kG, TargetAngle
   *
   * @return the command used to tune
   */
  public Command tune() {
    TunableConstant kP = new TunableConstant("/ElevatorArm/kP", config.kP());
    TunableConstant kI = new TunableConstant("/ElevatorArm/kI", config.kI());
    TunableConstant kD = new TunableConstant("/ElevatorArm/kD", config.kD());
    TunableConstant kG = new TunableConstant("/ElevatorArm/kG", config.kG());
    TunableConstant targetAngle = new TunableConstant("/ElevatorArm/TargetAngle", 0);
    TunableConstant maxVelocity = new TunableConstant("/ElevatorArm/MaxVelocity", 0);
    TunableConstant maxAcceleration = new TunableConstant("/ElevatorArm/MaxAcceleration", 0);

    return run(
        () -> {
          this.pidController.setPID(kP.get(), kI.get(), kD.get());
          this.feedforward = new ArmFeedforward(0, kG.get(), 0);
          this.profile =
              new TrapezoidProfile(
                  new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
          goalAngle = targetAngle.get();
          goToAngle(Degrees.of(targetAngle.get()));
        });
  }

  /** periodic method of the ElevatorArm Just updates the inputs from the sensors for now */
  public void periodic() {
    io.updateInputs(this.inputs);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        setpointState.position, goalAngle, ElevatorArmConstants.kAngleTolerance.in(Degrees));
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public double getSetpoint() {
    return setpointState.position;
  }

  public boolean atAngle(Angle angle) {
    return Math.abs(inputs.angle.in(Degrees) - angle.in(Degrees))
        < ElevatorArmConstants.kAngleTolerance.in(Degrees);
  }
}
