/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

// list of constants for the coral end effector
public class CoralEndEffectorConstants {
  // Motor configuration
  public static final int kMotorPort = 16;
  public static final int kCoralSensorId = 21;
  public static final boolean kInvertedMotor = true;
  public static final int kCurrentLimit = 40;

  // Physical constants
  public static final double kMomentOfInertia = 0.01;
  public static final double kGearing = 1;

  // Setpoints
  public static final AngularVelocity kCoralIntakeRPM = RPM.of(3000);
  public static final AngularVelocity kCoralOuttakeRPM = RPM.of(-1500);
  public static final AngularVelocity kCoralStallRPM = RPM.of(500);
  public static final AngularVelocity kAlgaeKnockRPM = RPM.of(-3000);

  // Tuned constants
  public static final Distance kDetectionRange = Millimeters.of(100);

  // for detecting if the end effector is intaking
  public static final AngularVelocity kRPMTolerance = RPM.of(240);
}
