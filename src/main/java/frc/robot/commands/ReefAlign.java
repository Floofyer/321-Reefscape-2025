/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDrive.AlignmentSetpoint;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.MyAlliance;
import frc.robot.util.ReefPosition;
import frc.robot.util.TunableConstant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ReefAlign {
  /*
    Maps reef AprilTag ("tag") ID to left, center, and right alignment poses,
    loads only the tags on alliance reef as determined at robot initialization
  */
  public static final Map<Integer, Pose2d> leftAlignPoses = new HashMap<>();
  public static final Map<Integer, Pose2d> centerAlignPoses = new HashMap<>();
  public static final Map<Integer, Pose2d> rightAlignPoses = new HashMap<>();

  private static final Distance kLeftAlignDistance = Inches.of(-8.85); // -9.1
  private static final Distance kReefDistance = Inches.of(17.5);
  private static final Distance kRightAlignDistance = Inches.of(3.6); // 5.4
  private static final Distance kIntermediateDistance = Inches.of(-10);

  private static final Rotation2d kReefAlignmentRotation = Rotation2d.k180deg;
  private static final Transform2d kLeftAlignTransform =
      new Transform2d(kReefDistance, kLeftAlignDistance, kReefAlignmentRotation);
  private static final Transform2d kCenterAlignTransform =
      new Transform2d(kReefDistance, Meter.zero(), kReefAlignmentRotation);
  private static final Transform2d kRightAlignTransform =
      new Transform2d(kReefDistance, kRightAlignDistance, kReefAlignmentRotation);

  private static final List<Integer> kBlueReefTagIDs = List.of(17, 18, 19, 20, 21, 22);
  private static final List<Integer> kRedReefTagIDs = List.of(6, 7, 8, 9, 10, 11);

  private static final List<Pose2d> blueReefTags = AprilTagUtil.tagIDsToPoses(kBlueReefTagIDs);
  private static final List<Pose2d> redReefTags = AprilTagUtil.tagIDsToPoses(kRedReefTagIDs);

  // TODO: use units
  public static final Pose2d kRedCenterAlignPos = new Pose2d(13, 4, Rotation2d.kZero);
  public static final Pose2d kBlueCenterAlignPos = new Pose2d(4.457, 4, Rotation2d.kZero);

  public static final Distance kMechanismDeadbandThreshold =
      Meters.of(2); // distance to trigger mechanism
  public static final Distance kMaxAlignmentDeadbandThreshold =
      Meters.of(5); // distance to trigger alignment

  /**
   * This method is run when DriverStation connects to save computations mid-match, only the poses
   * on the alliance reef are loaded
   */
  public static void loadReefAlignmentPoses() {
    for (int i = 0; i < kBlueReefTagIDs.size(); i++) {
      int blueTagID = kBlueReefTagIDs.get(i);
      int redTagID = kRedReefTagIDs.get(i);

      leftAlignPoses.computeIfAbsent(blueTagID, ReefAlign::getNearestLeftAlign);
      centerAlignPoses.computeIfAbsent(blueTagID, ReefAlign::getNearestCenterAlign);
      rightAlignPoses.computeIfAbsent(blueTagID, ReefAlign::getNearestRightAlign);

      leftAlignPoses.computeIfAbsent(redTagID, ReefAlign::getNearestLeftAlign);
      centerAlignPoses.computeIfAbsent(redTagID, ReefAlign::getNearestCenterAlign);
      rightAlignPoses.computeIfAbsent(redTagID, ReefAlign::getNearestRightAlign);
    }
  }

  /**
   * Finds the pose of the nearest reef tag on the alliance reef
   *
   * @param robotPose the pose of the robot to find the nearest reef tag pose for
   * @return null if robot alliance is unknown, otherwise a valid reef tag pose
   */
  public static Pose2d getNearestReefPose(Pose2d robotPose) {
    // `Optional` means the Alliance may not exist yet, which must be handled to proceed
    final Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    final List<Pose2d> tagsToCheck =
        alliance.get().equals(Alliance.Red) ? redReefTags : blueReefTags;

    return robotPose.nearest(tagsToCheck);
  }

  public static Pose2d getNearestReefPoseFullField(Pose2d robotPose) {
    // `Optional` means the Alliance may not exist yet, which must be handled to proceed
    final Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    final List<Pose2d> tagsToCheck = new ArrayList<>();
    tagsToCheck.addAll(redReefTags);
    tagsToCheck.addAll(blueReefTags);

    return robotPose.nearest(tagsToCheck);
  }

  /**
   * Finds the ID of the nearest reef tag on the alliance reef
   *
   * @param robotPose the pose of the robot to find the nearest reef tag ID for
   * @return -1 if robot alliance is unknown, otherwise a valid reef tag ID
   */
  public static int getNearestReefID(Pose2d robotPose) {
    Pose2d nearest = getNearestReefPose(robotPose);

    // handle the null that getNearestReefPose() may return when robot alliance is unknown
    if (nearest == null) return -1;

    return RobotConstants.kAprilTagFieldLayout.getTags().stream()
        .filter(tag -> tag.pose.toPose2d().equals(nearest))
        .toList()
        .get(0)
        .ID;
  }

  /**
   * Intended for aligning to the reef for removing algae
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestCenterAlign(int reefTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose = aprilTagPose.plus(kCenterAlignTransform);

    return resultPose;
  }

  /**
   * Intended for aligning to the left side reef bars for scoring coral
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestLeftAlign(int reefTagID) {
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose = aprilTagPose.plus(kLeftAlignTransform);

    return resultPose;
  }

  /**
   * Intended for aligning to the right side reef bars for scoring coral
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestRightAlign(int reefTagID) {
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose = aprilTagPose.plus(kRightAlignTransform);

    return resultPose;
  }

  public static Command alignToReef(
      SwerveDrive swerveDrive, Supplier<ReefPosition> targetReefPosition) {
    return Commands.runOnce(() -> Leds.getInstance().isReefAligning = true)
        .andThen(
            swerveDrive.driveToFieldPose(
                () -> {
                  Pose2d target =
                      switch (targetReefPosition.get()) {
                        case ALGAE -> centerAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                        case LEFT -> leftAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                        case RIGHT -> rightAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                        default -> swerveDrive.getPose(); // more or less a no-op
                      };

                  return new AlignmentSetpoint(target, true);
                }))
        .finallyDo(() -> Leds.getInstance().isReefAligning = false);
  }

  public static Command alignToPrealignReef(
      SwerveDrive swerveDrive, Supplier<ReefPosition> targetReefPosition) {
    return Commands.runOnce(() -> Leds.getInstance().isReefAligning = true)
        .andThen(
            swerveDrive.driveToFieldPose(
                () -> {
                  Pose2d target =
                      switch (targetReefPosition.get()) {
                        case ALGAE -> centerAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                        case LEFT -> leftAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                        case RIGHT -> rightAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                        default -> swerveDrive.getPose(); // more or less a no-op
                      };

                  target =
                      target.plus(
                          new Transform2d(
                              new Translation2d(kIntermediateDistance, Meters.zero()),
                              Rotation2d.kZero));
                  return new AlignmentSetpoint(target, false);
                }))
        .finallyDo(() -> Leds.getInstance().isReefAligning = false);
  }

  public static Command alignToTag(
      SwerveDrive swerveDrive, Supplier<ReefPosition> targetReefPosition) {
    return swerveDrive.driveToFieldPose(
        () -> {
          Pose2d target =
              switch (targetReefPosition.get()) {
                case ALGAE -> centerAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                case LEFT -> leftAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                case RIGHT -> rightAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
                default -> swerveDrive.getPose(); // more or less a no-op
              };

          Translation2d translationError =
              swerveDrive.getPose().relativeTo(target).getTranslation();

          Pose2d newTarget =
              target.plus(new Transform2d(translationError.getX(), 0, Rotation2d.kZero));

          return new AlignmentSetpoint(newTarget, false);
        });
  }

  public static Command tuneAlignment(SwerveDrive swerveDrive) {
    TunableConstant depth = new TunableConstant("/ReefAlign/Depth", kReefDistance.in(Inch));
    TunableConstant side = new TunableConstant("/ReefAlign/Side", kLeftAlignDistance.in(Inch));

    return swerveDrive.driveToFieldPose(
        () -> {
          Pose2d pose =
              getNearestReefPose(swerveDrive.getPose())
                  .transformBy(
                      new Transform2d(
                          Inches.of(depth.get()), Inches.of(side.get()), kReefAlignmentRotation));
          return new AlignmentSetpoint(pose, true);
        });
  }

  /** Maintain translational driving while rotating toward the nearest reef tag */
  public static Command rotateToNearestReefTag(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return Commands.runOnce(() -> Leds.getInstance().isRotateAligning = true)
        .andThen(
            swerveDrive.driveFixedHeading(
                x,
                y,
                () ->
                    getNearestReefPose(swerveDrive.getPose())
                        .getRotation()
                        .plus(kReefAlignmentRotation)))
        .finallyDo(() -> Leds.getInstance().isRotateAligning = false);
  }

  public static Command rotateToNearestReefTagFullField(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return Commands.runOnce(() -> Leds.getInstance().isRotateAligning = true)
        .andThen(
            swerveDrive.driveFixedHeading(
                x,
                y,
                () ->
                    getNearestReefPoseFullField(swerveDrive.getPose())
                        .getRotation()
                        .plus(kReefAlignmentRotation)))
        .finallyDo(() -> Leds.getInstance().isRotateAligning = false);
  }

  // if robot is within 2 meters of either red or blue reef, auto-align will NOT work
  public static boolean isWithinReefRange(SwerveDrive drive, Distance deadband) {
    Pose2d centerPos = MyAlliance.isRed() ? kRedCenterAlignPos : kBlueCenterAlignPos;
    double deadbandDistance =
        Math.hypot(
            drive.getPose().getX() - centerPos.getX(), drive.getPose().getY() - centerPos.getY());

    return deadbandDistance < deadband.in(Meters);
  }
}
