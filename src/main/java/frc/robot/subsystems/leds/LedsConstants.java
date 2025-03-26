/* (C) Robolancers 2025 */
package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;

@Logged
public class LedsConstants {
  public static final int kPort = 1;
  public static final int kLength = 30;
  public static final Time kBlinkSpeed = Seconds.of(0.3);
  public static final Color kAlgaeColor = new Color(133, 226, 203);

  // DRIVING PATTERNS
  // default mode - meteor yellow. TODO: add meteor pattern or something cool like that
  public static final LEDPattern kDefault = LEDPattern.solid(Color.kWhite).breathe(Seconds.of(5));

  // climb mode - solid blue
  public static final LEDPattern kClimbing = LEDPattern.solid(Color.kBlue);

  // when we are far away and only aligning rotationally - solid red
  public static final LEDPattern kRotationAligning = LEDPattern.solid(Color.kRed);

  // when the robot has a pose to align to - solid yellow
  public static final LEDPattern kReadyToAlign = LEDPattern.solid(Color.kYellow);

  public static final LEDPattern kReefAligned = LEDPattern.solid(Color.kGreen).blink(kBlinkSpeed);

  // when the robot is aligning to a pose - progress bar
  public static final LEDPattern kReefAligning(DoubleSupplier supp) {
    return LEDPattern.gradient(
            GradientType.kDiscontinuous,
            Color.kRed,
            Color.kOrange,
            Color.kYellow,
            Color.kGreenYellow,
            Color.kGreen)
        .mask(LEDPattern.progressMaskLayer(supp));
  }

  // when the driver interrupts the aligning process
  public static final LEDPattern kAlignOverride = LEDPattern.solid(Color.kPurple);

  // aligned and ready to score - solid green
  public static final LEDPattern kAligned = LEDPattern.solid(Color.kGreen);

  // has coral - solid white
  public static final LEDPattern kHasCoral = LEDPattern.solid(Color.kWhite);

  // has algae - solid algaeColor
  public static final LEDPattern kHasAlgae = LEDPattern.solid(kAlgaeColor);

  // has coral and algae - gradient of pink & light green (patrick star)
  public static final LEDPattern kHasCoralAndAlgae =
      LEDPattern.gradient(
              LEDPattern.GradientType.kContinuous, Color.kHotPink, new Color(172, 220, 65))
          .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75), Meters.of(1.5));

  // Intaking - solid orange
  public static final LEDPattern kIntaking = LEDPattern.solid(Color.kOrange);

  // Outtaking - blinking orange
  public static final LEDPattern kOuttaking = kIntaking.blink(kBlinkSpeed);

  // progress bar for aligning w/ aligning colors

  // ERROR STATE Patterns
  public static final LEDPattern kVisionDisconnect =
      LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.one(), Meters.one());

  public static final LEDPattern kRobotDisconnect = LEDPattern.solid(Color.kRed).blink(kBlinkSpeed);
}
