/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;

/*
 * Spark implementation of the real climber subsystem
 */

@Logged
public class ClimberIOSpark implements ClimberIO {

  public static final ClimberConfig config = new ClimberConfig(2, 0, 0, 0);

  private SparkMax climbMotor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);

  private final Servo climbServo = new Servo(ClimberConstants.kServoPort);

  public void configureMotors() {
    climbMotor.configure( // configures single motor
        new SparkMaxConfig()
            .inverted(ClimberConstants.kClimbInverted)
            .voltageCompensation(ClimberConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(ClimberConstants.kSmartCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ClimberConstants.kClimbVelocityConversionFactor)
                    .positionConversionFactor(ClimberConstants.kClimbPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public ClimberIOSpark() {
    configureMotors();
  }

  // sets climb current
  public void setClimbCurrent(Current current) {
    climbMotor.getClosedLoopController().setReference(current.in(Amps), ControlType.kCurrent);
  }

  // sets climb voltage
  public void setClimbVoltage(Voltage volts) {
    climbMotor.setVoltage(volts);
  }

  // sets servo to a specified position
  public void setLockServoAngle(Angle angle) {
    climbServo.setAngle(angle.in(Degrees));
  }

  public void updateInputs(ClimberInputs inputs) {
    // Gets raw angle from the encoder
    double rawAngle = climbMotor.getEncoder().getPosition();

    // Update inputs with the modulus-adjusted angle
    inputs.climbAngle = Degrees.of(rawAngle);
    inputs.climbVelocity = DegreesPerSecond.of(climbMotor.getEncoder().getVelocity());
    inputs.climbCurrent = Amps.of(climbMotor.getOutputCurrent());
  }

  public void resetEncoder(Angle angle) {
    climbMotor.getEncoder().setPosition(angle.in(Degrees));
  }
}
