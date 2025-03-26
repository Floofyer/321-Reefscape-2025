/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

// coral end effector subsystem (now a coral / algae end effector mechanism)
@Logged
public class CoralEndEffector extends SubsystemBase {
  private CoralEndEffectorInputs inputs;
  private CoralEndEffectorIO io;

  private PIDController endEffectorController;
  private SimpleMotorFeedforward feedforward;

  private CoralEndEffectorConfig config;

  public static CoralEndEffector create() {
    return RobotBase.isReal()
        ? new CoralEndEffector(new CoralEndEffectorIOReal(), CoralEndEffectorIOReal.config)
        : new CoralEndEffector(new CoralEndEffectorIOSim(), CoralEndEffectorIOSim.config);
  }

  public static CoralEndEffector disable() {
    return new CoralEndEffector(new CoralEndEffectorIOIdeal(), CoralEndEffectorIOIdeal.config);
  }

  public CoralEndEffector(CoralEndEffectorIO io, CoralEndEffectorConfig config) {
    this.io = io;
    this.inputs = new CoralEndEffectorInputs();
    this.config = config;
    this.endEffectorController = new PIDController(config.kP(), config.kI(), config.kD());
    this.feedforward = new SimpleMotorFeedforward(0, config.kV());
  }

  // constantly updates inputs
  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  // run the end effector at a certain specified velocity using PIDFF control
  // Will only run once; For a continuous method, see runAtVelocity(Supplier<AngularVelocity>)
  public void runAtVelocity(AngularVelocity velocity) {
    double output =
        endEffectorController.calculate(inputs.velocity.in(RPM), velocity.in(RPM))
            + feedforward.calculate(velocity.in(RPM));
    io.setVoltage(Volts.of(output));
  }

  // continuously run the end effector at a certain velocity supplied by the velocity supplier
  public Command runAtVelocity(Supplier<AngularVelocity> velocity) {
    return run(
        () -> {
          runAtVelocity(velocity.get());
        });
  }

  // shortcut to intake coral
  public Command intakeCoral() {
    return Commands.runOnce(() -> Leds.getInstance().isIntaking = true)
        .andThen(runAtVelocity(() -> CoralEndEffectorConstants.kCoralIntakeRPM))
        .finallyDo(() -> Leds.getInstance().isIntaking = false);
  }

  // shortcut to outtake coral
  public Command outtakeCoral() {
    return Commands.runOnce(() -> Leds.getInstance().isOuttaking = true)
        .andThen(runAtVelocity(() -> CoralEndEffectorConstants.kCoralOuttakeRPM))
        .finallyDo(() -> Leds.getInstance().isOuttaking = false);
  }

  public Command runVolts(Supplier<Voltage> voltage) {
    return run(() -> io.setVoltage(voltage.get()));
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  // stalls coral if we have a coral; this should be the default command
  public Command stallCoralIfDetected() {
    return runAtVelocity(
        () -> {
          if (hasCoral()) {
            return CoralEndEffectorConstants.kCoralStallRPM;
          }
          return RPM.of(0);
        });
  }

  // tune PIDFF of end effector
  public Command tune() {
    TunableConstant kP = new TunableConstant("/CoralEndEffector/kP", config.kP());
    TunableConstant kI = new TunableConstant("/CoralEndEffector/kI", config.kI());
    TunableConstant kD = new TunableConstant("/CoralEndEffector/kD", config.kD());
    TunableConstant kV = new TunableConstant("/CoralEndEffector/kV", config.kV());
    TunableConstant targetRPM = new TunableConstant("/CoralEndEffector/TargetRPM", 0);

    return run(
        () -> {
          endEffectorController.setPID(kP.get(), kI.get(), kD.get());
          feedforward = new SimpleMotorFeedforward(0, kV.get());
          runAtVelocity(RPM.of(targetRPM.get()));
        });
  }
}
