/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.RobotConstants;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  // TODO: tune more thoroughly
  public static final double kTranslationStdDevCoeff = 1e-1;
  public static final double kRotationStdDevCoeff = 1e-1;

  public static record CameraCalibration(
      int resolutionWidth,
      int resolutionHeight,
      Rotation2d fovDiag,
      double avgErrorPx,
      double errorStdDevPx,
      Time exposureTime,
      double fps,
      Time avgLatency,
      Time latencyStdDev) {
    public SimCameraProperties simProperties() {
      final var simProps = new SimCameraProperties();

      simProps.setCalibration(resolutionWidth, resolutionHeight, fovDiag);
      simProps.setCalibError(avgErrorPx, errorStdDevPx);

      simProps.setExposureTimeMs(exposureTime.in(Milliseconds));
      simProps.setFPS(fps);

      simProps.setAvgLatencyMs(avgLatency.in(Milliseconds));
      simProps.setLatencyStdDevMs(latencyStdDev.in(Milliseconds));

      return simProps;
    }
  }

  public static final CameraCalibration kOV9281 =
      new CameraCalibration(
          1280,
          720,
          Rotation2d.fromDegrees(70),
          // TODO: find actual values for this from calibration in Photon Client
          // calibration file does not show these values and config.json mentioned in docs appears
          // inaccessibles
          0.25,
          0.08,
          Seconds.zero(),
          30,
          Milliseconds.of(35),
          Milliseconds.of(5));

  public static record CameraConfig(
      String cameraName, CameraUsage usage, Transform3d robotToCamera, CameraCalibration calib) {}

  private static final Transform3d k427CameraMountTransform =
      new Transform3d(
          Meters.of(-0.27),
          Meters.zero(),
          Meters.zero(),
          new Rotation3d(Degrees.zero(), Degrees.of(30), Degrees.of(180)));

  private static final Transform3d k321TopElevatorCameraMountTransform =
      new Transform3d(
          Meters.of(0.2314956),
          Meters.of(-0.16764),
          Meters.of(0.3103626),
          new Rotation3d(Degrees.zero(), Degrees.of(-1), Degrees.of(48)));

  private static final Transform3d k321BottomElevatorCameraMountTransform =
      new Transform3d(
          Meters.of(0.2280412),
          Meters.of(-0.1723644),
          Meters.of(0.2151634),
          new Rotation3d(Degrees.zero(), Degrees.of(-18), Degrees.of(-10)));

  private static final Transform3d k321FrontSwerveModuleCameraMountTransform =
      new Transform3d(
          Meters.of(-0.2290318),
          Meters.of(0.322326),
          Meters.of(0.1966722),
          new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(90)));
  // new Transform3d(Meters.of(0.322326), Meters.of(0.2290318), Meters.of(0.1966722), new
  // Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.zero()));

  private static final Transform3d k321BackLeftSwerveModuleCameraMountTransform =
      new Transform3d(
          Meters.of(-0.2275078),
          Meters.of(-0.1823466),
          Meters.of(0.2745486),
          new Rotation3d(Degrees.zero(), Degrees.of(-12), Degrees.of(225)));
  //  new Transform3d(Meters.of(-0.2278126), Meters.of(0.3010408), Meters.of(0.1971802), new
  // Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(135)));

  public static final CameraConfig kElevatorTopCameraConfig =
      new CameraConfig(
          "Top Elevator Camera", CameraUsage.REEF, k321TopElevatorCameraMountTransform, kOV9281);

  public static final CameraConfig kElevatorBottomCameraConfig =
      new CameraConfig(
          "Bottom Elevator Camera",
          CameraUsage.REEF,
          k321BottomElevatorCameraMountTransform,
          kOV9281);

  public static final CameraConfig kFrontSwerveCameraConfig =
      new CameraConfig(
          "Front Swerve Module Camera",
          CameraUsage.GENERAL,
          k321FrontSwerveModuleCameraMountTransform,
          kOV9281);

  public static final CameraConfig kBackLeftSwerveCameraConfig =
      new CameraConfig(
          "Back Left Swerve Module Camera",
          CameraUsage.GENERAL,
          k321BackLeftSwerveModuleCameraMountTransform,
          kOV9281);

  public static final CameraConfig[] kCameraConfigs = {
    kElevatorTopCameraConfig, kElevatorBottomCameraConfig
    // kFrontSwerveCameraConfig,
    // kBackLeftSwerveCameraConfig
  };

  // camera data filtering
  public static final Distance kAllowedFieldDistance =
      Meters.of(2.5); // allow field estimates 2.5 meters outside field
  public static final Distance kAllowedFieldHeight =
      Meters.of(0.75); // Vision estimates can be at maximum 0.75 meters off the floor before being
  // rejected
  public static final Rectangle2d kAllowedFieldArea =
      new Rectangle2d(
          new Translation2d(-kAllowedFieldDistance.in(Meters), -kAllowedFieldDistance.in(Meters)),
          new Translation2d(
              RobotConstants.kAprilTagFieldLayout.getFieldLength()
                  + kAllowedFieldDistance.in(Meters),
              RobotConstants.kAprilTagFieldLayout.getFieldWidth()
                  + kAllowedFieldDistance.in(Meters)));
}
