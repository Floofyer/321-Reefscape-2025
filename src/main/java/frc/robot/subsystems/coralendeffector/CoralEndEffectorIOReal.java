/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// implementation of the CoralEndEffectorIO that controls the real coral end effector using a
// SparkMax motoro
@Logged
public class CoralEndEffectorIOReal implements CoralEndEffectorIO {

  public static CoralEndEffectorConfig config = new CoralEndEffectorConfig(0.00007, 0, 0, 0.0022);

  private SparkMax motor; // motor controlling the end effector wheels
  private TimeOfFlight
      coralDistSensor; // TOF distance sensor for detecting whether or not there is a coral in the

  // intake

  public CoralEndEffectorIOReal() {
    // motor initializatio & configuration
    this.motor = new SparkMax(CoralEndEffectorConstants.kMotorPort, MotorType.kBrushless);
    motor.configure(
        new SparkMaxConfig()
            .inverted(CoralEndEffectorConstants.kInvertedMotor)
            .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // init distance sensor
    coralDistSensor = new TimeOfFlight(CoralEndEffectorConstants.kCoralSensorId);
  }

  // sets voltage of the coral intake wheels
  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  // update inputs from the coral intake sensors
  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = Volts.of(motor.getBusVoltage());
    inputs.hasCoral =
        coralDistSensor.getRange() < CoralEndEffectorConstants.kDetectionRange.in(Millimeter);
    inputs.velocity = RPM.of(this.motor.getEncoder().getVelocity());
  }
}
