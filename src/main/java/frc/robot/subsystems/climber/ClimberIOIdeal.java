/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// implementation of ClimberIO when disabled
@Logged
public class ClimberIOIdeal implements ClimberIO {

  public static final ClimberConfig config = new ClimberConfig(0, 0, 0, 0);

  public void updateInputs(ClimberInputs inputs) {
    inputs.climbAngle = Degrees.of(0);
    inputs.climbVelocity = DegreesPerSecond.of(0);
    inputs.climbCurrent = Amps.zero();
  }

  public void setClimbVoltage(Voltage volts) {}

  public void setClimbCurrent(Current current) {}
}
