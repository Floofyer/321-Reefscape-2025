/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

@Logged
public class CoralSuperstructure {

  @NotLogged private Elevator elevator;
  @NotLogged private ElevatorArm arm;
  @NotLogged private CoralEndEffector endEffector;

  private Distance targetHeight = CoralScorerSetpoint.NEUTRAL.getElevatorHeight();
  private Angle targetAngle = CoralScorerSetpoint.NEUTRAL.getArmAngle();

  public CoralSuperstructure(Elevator elevator, ElevatorArm arm, CoralEndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
  }

  // moves the entire elevator+arm superstructure to a desired state; this should be the go-to way
  // of moving the superstructure, aside from the default subsystem commands
  public Command goToSetpointPID(Supplier<CoralScorerSetpoint> setpoint) {
    return goToSetpointPID(
        () -> setpoint.get().getElevatorHeight(), () -> setpoint.get().getArmAngle());
  }

  public Command goToSetpointPID(Supplier<Distance> height, Supplier<Angle> angle) {
    return elevator
        .goToHeight(() -> height.get())
        .alongWith(arm.goToAnglePID(() -> angle.get()))
        .deadlineFor(
            Commands.run(
                () -> {
                  targetHeight = height.get();
                  targetAngle = angle.get();
                }));
  }

  public Command goToSetpointProfiled(Supplier<CoralScorerSetpoint> setpoint) {
    return goToSetpointProfiled(
        () -> setpoint.get().getElevatorHeight(), () -> setpoint.get().getArmAngle());
  }

  public Command goToSetpointProfiled(Supplier<Distance> height, Supplier<Angle> angle) {
    return elevator
        .goToHeight(height)
        .alongWith(arm.goToAngleProfiled(() -> angle.get()))
        .deadlineFor(
            Commands.run(
                () -> {
                  targetHeight = height.get();
                  targetAngle = angle.get();
                }));
  }

  public Command stopIntake() {
    return endEffector.runAtVelocity(() -> RPM.zero());
  }

  public void goToSetpoint(CoralScorerSetpoint setpoint) {
    goToSetpoint(setpoint.getElevatorHeight(), setpoint.getArmAngle());
  }

  public void goToSetpoint(Distance height, Angle ang) {
    elevator.goToHeight(height);
    arm.goToAngle(ang);
  }

  public boolean atTargetState() {
    return elevator.atSetpoint() && arm.atGoal();
  }

  public boolean atTargetState(Distance height, Angle angle) {
    return elevator.atHeight(height) && arm.atAngle(angle);
  }

  public boolean atTargetState(CoralScorerSetpoint setpoint) {
    return atTargetState(setpoint.getElevatorHeight(), setpoint.getArmAngle());
  }

  public Command feedCoral() {
    return goToSetpointPID(() -> CoralScorerSetpoint.FEED_CORAL)
        .alongWith(endEffector.intakeCoral());
  }

  public Command outtakeCoral() {
    return endEffector.outtakeCoral();
  }

  public Command knockAlgae() {
    return endEffector.runAtVelocity(() -> CoralEndEffectorConstants.kAlgaeKnockRPM);
  }

  public Distance getTargetHeight() {
    return targetHeight;
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  public boolean hasCoral() {
    return endEffector.hasCoral();
  }

  public Command tune() {
    TunableConstant armAngle =
        new TunableConstant(
            "/CoralSuperstructure/ArmAngle", CoralScorerSetpoint.NEUTRAL.getArmAngle().in(Degrees));
    TunableConstant height =
        new TunableConstant(
            "/CoralSuperstructure/ElevatorHeight",
            CoralScorerSetpoint.L4.getElevatorHeight().in(Meters));

    return arm.goToAnglePID(() -> CoralScorerSetpoint.PREALIGN.getArmAngle())
        .until(arm::atSetpoint)
        .andThen(elevator.goToHeight(() -> Meters.of(height.get())).until(elevator::atSetpoint))
        .andThen(arm.goToAngleProfiled(() -> Degrees.of(armAngle.get())));
  }

  @NotLogged
  public Elevator getElevator() {
    return elevator;
  }

  @NotLogged
  public CoralEndEffector getEndEffector() {
    return endEffector;
  }

  public enum CoralScorerSetpoint {
    // TODO: determine angles empirically
    NEUTRAL(ElevatorConstants.kElevatorStartingHeight.plus(Meters.of(0.1)), Degrees.of(-40)),
    FEED_CORAL(Meters.of(0.885), Degrees.of(-87)),
    L1(Inches.of(45), Degrees.of(30)),
    L2(Meters.of(0.96), Degrees.of(95)),
    L3(Meters.of(1.3).plus(Inches.of(1.25)), Degrees.of(95)),
    L4(Meters.of(2.06).plus(Inches.of(0.5)), Degrees.of(85)),
    ALGAE_LOW(Meters.of(1), Degrees.of(40)),
    ALGAE_HIGH(Meters.of(1.4), Degrees.of(40)),
    PREALIGN(Inches.of(55), Degrees.of(120)),
    CLIMB(Meters.of(1.1), Degrees.of(0));

    private Distance elevatorHeight; // the height of the elevator to got
    private Angle armAngle; // the angle the arm should go to

    CoralScorerSetpoint(Distance elevatorHeight, Angle armAngle) {
      this.armAngle = armAngle;
      this.elevatorHeight = elevatorHeight;
    }

    public Distance getElevatorHeight() {
      return elevatorHeight;
    }

    public Angle getArmAngle() {
      return armAngle;
    }
  }
}
