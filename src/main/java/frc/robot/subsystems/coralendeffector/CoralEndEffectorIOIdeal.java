/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;

@Logged
// implementation of CoralEndEffectorIO that disables the end effector
public class CoralEndEffectorIOIdeal implements CoralEndEffectorIO {

  public static CoralEndEffectorConfig config = new CoralEndEffectorConfig(0, 0, 0, 0);

  public CoralEndEffectorIOIdeal() {}

  // All input default values are off or false
  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = Volts.of(0);
    inputs.hasCoral = false;
    inputs.velocity = RPM.of(0);
  }
}
