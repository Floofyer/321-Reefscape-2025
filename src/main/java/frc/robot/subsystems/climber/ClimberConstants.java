/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ClimberConstants {
  // climb motor IDs
  public static final int kMotorId = 20; // placeholder id
  public static final int kServoPort = 0;
  public static final int kLimitSwitchPort = 0;

  // climb physical constants
  public static final double kClimbGearing = 300;
  public static final double kClimbMOI = 0.1;
  public static final Angle kStartingAngle = Degrees.of(-90);
  public static final Distance kClimbLength = Inches.of(0.6);
  public static final Angle kClimbMinAngle = Degrees.of(-90); // freely rotating climber
  public static final Angle kClimbMaxAngle = Degrees.of(270);
  public static final Angle kClimbPrepAngle = Degrees.of(180);
  public static final Angle kDefaultAngle = Degrees.of(-90);

  // climb servo lock + unlock positions
  public static final Angle kServoLockPosition = Degrees.of(90);
  public static final Angle kServoUnlockPosition = Degrees.of(180);

  // climb motor config
  public static final boolean kClimbInverted = false;
  public static final int kSmartCurrentLimit = 60;
  public static final double kClimbPositionConversionFactor = 360 / kClimbGearing;
  public static final double kClimbVelocityConversionFactor = kClimbPositionConversionFactor / 60;
  public static final Voltage kClimbHomeVoltage = Volts.of(-3);

  public static final Current kClimbHomeCurrent = Amps.of(30);

  public static final Voltage kNominalVoltage = Volts.of(12);
  public static final Angle kClimbThreshold = Degrees.of(0); // to be tuned

  public static final Voltage kClimbVoltageRampRate =
      Volts.of(3); // placeholder, will need to tune this
  public static final Voltage kClimbVoltage = Volts.of(12); // placeholder

  public static final Angle kControllerTolerance = Degrees.of(1);

  // field constants for climbing
  // Coordinates of barge safety area (including pibot from center of robot)
  public static final Translation2d kBargeRightCorner = new Translation2d(10, 8);
  public static final Translation2d kBargeLeftCorner = new Translation2d(7.6, 0);
  // Rectangle of barge area
  public static final Rectangle2d kBargeZone = new Rectangle2d(kBargeLeftCorner, kBargeRightCorner);
}
