/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

// below are inputs that the coral end effector will receive
@Logged
public class CoralEndEffectorInputs {
  public Voltage voltage; // voltage the end effector motor is currently being commanded to run at
  public boolean
      hasCoral; // whether or not the end effector has a coral in it, measured by a distance sensor
  public AngularVelocity velocity; // the angular velocity of the end effector wheels
}
