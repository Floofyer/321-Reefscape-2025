/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class DrivetrainConstants {
  public record AutoGains(double kP, double kI, double kD) {}

  public static final AutoGains kTranslationGains =
      RobotBase.isReal()
          ? new AutoGains(6.328, 0, 0.01) // real (TODO: TUNE)
          : new AutoGains(6.328, 0, 0); // sim

  public static final Constraints kTranslationConstraints =
      RobotBase.isReal() ? new Constraints(4.5, 5) : new Constraints(4.5, 5);

  public static final AutoGains kHeadingGains =
      RobotBase.isReal()
          ? new AutoGains(3.14, 0, 0) // real (TODO: TUNE)
          : new AutoGains(3.14, 0, 0); // sim

  public static final Constraints kHeadingConstraints =
      RobotBase.isReal() ? new Constraints(6 * Math.PI, 12) : new Constraints(2 * Math.PI, 8);

  public static final AutoGains kTuneTranslationGains = new AutoGains(0, 0, 0); // isn't used
  public static final AutoGains kTuneHeadingGains =
      new AutoGains(6, 0, 0.01); // for heading controller

  public static final Distance kTrackWidth = Inches.of(29);
  public static final Distance kWheelBase = Inches.of(29);

  public static final double kDriveDeadband = 0.05;
  public static final double kRotationDeadband = 0.05;
  public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond.of(6);
  public static final LinearVelocity kMaxLinearVelocity =
      MetersPerSecond.of(5.0); // TunerConstants.kSpeedAt12Volts

  public static final Distance kAlignmentSetpointTranslationTolerance = Meters.of(0.01);
  public static final Angle kAlignmentSetpointRotationTolerance = Degrees.of(2.0);
}
