/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotConstants {
  public static final Time kRobotLoopPeriod = Seconds.of(TimedRobot.kDefaultPeriod);

  public static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final Pose2d kRedCenterAlignPos = new Pose2d(13, 4, new Rotation2d());
  public static final Pose2d kBlueCenterAlignPos = new Pose2d(4.457, 4, new Rotation2d());

  public static final Distance kMechanismDeadbandThreshold =
      Meters.of(2); // distance to trigger mechanism
  public static final Distance kMaxAlignmentDeadbandThreshold =
      Meters.of(5); // distance to trigger alignment
}
