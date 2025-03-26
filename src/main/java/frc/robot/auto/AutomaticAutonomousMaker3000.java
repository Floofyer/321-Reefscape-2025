/* (C) Robolancers 2025 */
package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.StationAlign;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.util.ReefPosition;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;

@Logged
public class AutomaticAutonomousMaker3000 {

  private static final LinearVelocity kScorePathEndVelocity = MetersPerSecond.of(0);

  private CycleAutoChooser autoChooser = new CycleAutoChooser(5);

  private Field2d field = new Field2d();
  private String pathError = "";
  private List<Pose2d> visualizePath = new ArrayList<>();
  private SendableChooser<PreBuiltAuto> preBuiltAuto = new SendableChooser<>();

  private static CycleAutoConfig kTopLaneAuto =
      new CycleAutoConfig(
          StartingPosition.TOP,
          List.of(
              new ScoringGroup(
                  FeedLocation.UPCORALRIGHT, ReefSide.REEFR1, Pole.RIGHTPOLE, Level.L4),
              new ScoringGroup(FeedLocation.UPCORALRIGHT, ReefSide.REEFL1, Pole.LEFTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.UPCORALRIGHT, ReefSide.REEFL1, Pole.RIGHTPOLE, Level.L4)));

  private static CycleAutoConfig kMidLaneTopAuto =
      new CycleAutoConfig(
          StartingPosition.MIDDLE,
          List.of(
              new ScoringGroup(
                  FeedLocation.UPCORALRIGHT, ReefSide.REEFR2, Pole.RIGHTPOLE, Level.L4),
              new ScoringGroup(FeedLocation.UPCORALRIGHT, ReefSide.REEFL1, Pole.LEFTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.UPCORALRIGHT, ReefSide.REEFL1, Pole.RIGHTPOLE, Level.L4)));

  private static CycleAutoConfig kMidLaneBotAuto =
      new CycleAutoConfig(
          StartingPosition.MIDDLE,
          List.of(
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFR2, Pole.LEFTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFL3, Pole.LEFTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFL3, Pole.RIGHTPOLE, Level.L4)));

  private static CycleAutoConfig kMidLaneBotPreloadAuto =
      new CycleAutoConfig(
          StartingPosition.MIDDLE,
          List.of(
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFR2, Pole.RIGHTPOLE, Level.L4)));

  private static CycleAutoConfig kMidLaneOppositeSideAuto =
      new CycleAutoConfig(
          StartingPosition.MIDDLE,
          List.of(
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFL2, Pole.LEFTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFL3, Pole.LEFTPOLE, Level.L4)));

  private static CycleAutoConfig kBotLaneAuto =
      new CycleAutoConfig(
          StartingPosition.BOTTOM,
          List.of(
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFR3, Pole.LEFTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFL3, Pole.RIGHTPOLE, Level.L4),
              new ScoringGroup(
                  FeedLocation.DOWNCORALLEFT, ReefSide.REEFL3, Pole.LEFTPOLE, Level.L4)));

  private SwerveDrive drive;
  private CoralSuperstructure coralSuperstructure;

  private Command storedAuto;

  public AutomaticAutonomousMaker3000(SwerveDrive drive, CoralSuperstructure coralSuperstructure) {
    this.drive = drive;
    this.coralSuperstructure = coralSuperstructure;

    preBuiltAuto.setDefaultOption("No Choice", PreBuiltAuto.CUSTOM);
    preBuiltAuto.addOption("Taxi", PreBuiltAuto.TAXI);
    preBuiltAuto.addOption("TopAuto", PreBuiltAuto.TOPAUTO);
    preBuiltAuto.addOption("MidTopAuto", PreBuiltAuto.MIDTOPAUTO);
    preBuiltAuto.addOption("MidBotAuto", PreBuiltAuto.MIDBOTAUTO);
    preBuiltAuto.addOption("BotAuto", PreBuiltAuto.BOTAUTO);
    preBuiltAuto.addOption("MidPreloadAuto", PreBuiltAuto.MIDPRELOADAUTO);
    preBuiltAuto.addOption("MidOppositeSideAuto", PreBuiltAuto.MIDOPPOSITESIDEAUTO);
    preBuiltAuto.addOption("Custom Auto", PreBuiltAuto.CUSTOM);

    SmartDashboard.putData("Autos/PreBuiltAuto", preBuiltAuto);
    SmartDashboard.putData("Autos/AutoVisualizerField", field);
    // SmartDashboard.putData("Autos/TotalTime", totalTime);

    // Driver has to click submit to make and view the autonomous path
    SmartDashboard.putData(
        "Autos/Submit",
        Commands.runOnce(
                () -> {

                  // Pre made autos first and then the custom autos
                  PathsAndAuto selectedAuto =
                      switch (preBuiltAuto.getSelected()) {
                        case TAXI ->
                            runPath(autoChooser.build().startingPosition.pathID + " to Brake");
                        case TOPAUTO -> buildAuto(kTopLaneAuto);
                        case MIDTOPAUTO -> buildAuto(kMidLaneTopAuto);
                        case MIDBOTAUTO -> buildAuto(kMidLaneBotAuto);
                        case BOTAUTO -> buildAuto(kBotLaneAuto);
                        case MIDPRELOADAUTO -> buildAuto(kMidLaneBotPreloadAuto); // test auto again
                        case MIDOPPOSITESIDEAUTO ->
                            buildAuto(kMidLaneOppositeSideAuto); // test auto x2
                        case CUSTOM -> buildAuto(autoChooser.build());
                        default -> new PathsAndAuto(Commands.none(), new ArrayList<>());
                      };

                  if (selectedAuto != null) {
                    storedAuto = selectedAuto.getAuto();
                    visualizeAuto(selectedAuto.getPaths());
                  }
                  // Clears the simulated field path
                  else {
                    visualizePath.clear();
                    UpdateFieldVisualization();
                  }
                  UpdatePathError();
                })
            .ignoringDisable(true)
            .withName("Submit Auto"));
  }

  private void UpdateFieldVisualization() {
    field.getObject("PathPoses").setPoses(visualizePath);
  }

  private void UpdatePathError() {
    SmartDashboard.putString("Autos/Path Error", pathError);
  }

  public Command getStoredAuto() {
    return storedAuto;
  }

  private PathsAndAuto runPath(String pathName) {
    try {
      PathPlannerPath path = getPath(pathName);

      if (path == null) return null;
      return new PathsAndAuto(toPathCommand(path, true), List.of(path));
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
      return null;
    }
  }

  private void visualizeAuto(List<PathPlannerPath> paths) {
    visualizePath.clear();

    for (int i = 0; i < paths.size(); i++) {
      visualizePath.addAll(paths.get(i).getPathPoses());
    }

    UpdateFieldVisualization();
  }

  // Returns the path list for visualization and autonomous command
  public PathsAndAuto buildAuto(CycleAutoConfig config) {
    pathError = "";
    try {
      Command auto =
          coralSuperstructure
              .getElevator()
              .homeEncoder()
              .onlyIf(() -> !coralSuperstructure.getElevator().elevatorIsHomed())
              .withTimeout(2);

      List<PathPlannerPath> paths = new ArrayList<>();

      ReefSide lastReefSide = config.scoringGroup.get(0).reefSide;

      for (int i = 0; i < config.scoringGroup.size(); i++) {

        if ((i != 0 && config.scoringGroup.get(i).feedLocation == FeedLocation.NOCHOICE)
            || config.scoringGroup.get(i).reefSide == ReefSide.NOCHOICE) break;
        if (i == 0) {
          // first value; score preload and ignore the alt destination instructions
          PathPlannerPath path =
              getPath(
                  config.startingPosition.pathID
                      + " to "
                      + config.scoringGroup.get(i).reefSide.pathID);

          PathPlannerPath pathNewGoalEndState =
              new PathPlannerPath(
                  path.getWaypoints(),
                  path.getRotationTargets(),
                  path.getPointTowardsZones(),
                  path.getConstraintZones(),
                  path.getEventMarkers(),
                  path.getGlobalConstraints(),
                  path.getIdealStartingState(),
                  new GoalEndState(kScorePathEndVelocity, path.getGoalEndState().rotation()),
                  path.isReversed());

          auto =
              auto.andThen(
                  withScoring(
                      toPathCommand(pathNewGoalEndState, true),
                      config.scoringGroup.get(i).pole,
                      config.scoringGroup.get(i).level));
          paths.add(path);
        } else {

          PathPlannerPath intakePath =
              getPath(
                  lastReefSide.pathID + " to " + config.scoringGroup.get(i).feedLocation.pathID);

          PathPlannerPath scorePath =
              getPath(
                  config.scoringGroup.get(i).feedLocation.pathID
                      + " to "
                      + config.scoringGroup.get(i).reefSide.pathID);

          PathPlannerPath scorePathNewGoalEndState =
              new PathPlannerPath(
                  scorePath.getWaypoints(),
                  scorePath.getRotationTargets(),
                  scorePath.getPointTowardsZones(),
                  scorePath.getConstraintZones(),
                  scorePath.getEventMarkers(),
                  scorePath.getGlobalConstraints(),
                  scorePath.getIdealStartingState(),
                  new GoalEndState(kScorePathEndVelocity, scorePath.getGoalEndState().rotation()),
                  scorePath.isReversed());

          auto =
              auto.andThen(
                      withIntaking(
                          toPathCommand(intakePath), config.scoringGroup.get(i).feedLocation))
                  .andThen(
                      withScoring(
                          toPathCommand(scorePathNewGoalEndState),
                          config.scoringGroup.get(i).pole,
                          config.scoringGroup.get(i).level));
          lastReefSide = config.scoringGroup.get(i).reefSide;

          paths.add(intakePath);
          paths.add(scorePath);
        }
      }

      auto =
          auto.andThen(
              drive
                  .driveFieldCentric(() -> 0, () -> 0, () -> 0)
                  .alongWith(
                      coralSuperstructure.goToSetpointPID(
                          () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                          () -> CoralScorerSetpoint.PREALIGN.getArmAngle()),
                      coralSuperstructure.stopIntake()));

      return new PathsAndAuto(auto, paths);
    } catch (Exception e) {
      pathError = "Path doesn't exist";
      return null;
    }
  }

  public Command withIntaking(Command path, FeedLocation location) {
    Command pathCmd =
        switch (location) {
          default -> StationAlign.goToNearestLeftAlign(drive);
          case UPCORALLEFT, DOWNCORALLEFT -> StationAlign.goToNearestLeftAlign(drive);
          case UPCORALMIDDLE, DOWNCORALMIDDLE -> StationAlign.goToNearestCenterAlign(drive);
          case UPCORALRIGHT, DOWNCORALRIGHT -> StationAlign.goToNearestRightAlign(drive);
        };

    return path.andThen(pathCmd)
        .withDeadline(
            coralSuperstructure
                .goToSetpointPID(
                    () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                    () -> CoralScorerSetpoint.PREALIGN.getArmAngle())
                .alongWith(coralSuperstructure.stopIntake())
                .until(
                    () ->
                        coralSuperstructure.atTargetState(
                            CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            CoralScorerSetpoint.PREALIGN.getArmAngle()))
                .andThen(
                    coralSuperstructure.feedCoral().until(() -> coralSuperstructure.hasCoral())));
  }

  public Command withScoring(Command path, Pole pole, Level level) {
    CoralScorerSetpoint setpoint =
        switch (level) {
          default -> CoralScorerSetpoint.L1;
          case L1 -> CoralScorerSetpoint.L1;
          case L2 -> CoralScorerSetpoint.L2;
          case L3 -> CoralScorerSetpoint.L3;
          case L4 -> CoralScorerSetpoint.L4;
        };

    Distance preAlignElevatorHeight =
        Meters.of(
            Math.min(
                CoralScorerSetpoint.PREALIGN.getElevatorHeight().in(Meters),
                setpoint.getElevatorHeight().in(Meters)));

    return path.deadlineFor(
            coralSuperstructure
                .goToSetpointProfiled(() -> preAlignElevatorHeight, () -> setpoint.getArmAngle())
                .alongWith(coralSuperstructure.getEndEffector().stallCoralIfDetected()))
        .andThen(
            ReefAlign.alignToReef(
                    drive, () -> pole == Pole.LEFTPOLE ? ReefPosition.LEFT : ReefPosition.RIGHT)
                .alongWith(
                    coralSuperstructure.goToSetpointPID(
                        () -> preAlignElevatorHeight, () -> setpoint.getArmAngle()),
                    coralSuperstructure.getEndEffector().stallCoralIfDetected())
                .until(() -> drive.atPoseSetpoint())
                .withTimeout(2.5)
                .andThen(
                    coralSuperstructure
                        .goToSetpointProfiled(() -> setpoint)
                        .alongWith(coralSuperstructure.getEndEffector().stallCoralIfDetected())
                        .until(() -> coralSuperstructure.atTargetState(setpoint))))
        .andThen(
            ReefAlign.alignToReef(
                    drive, () -> pole == Pole.LEFTPOLE ? ReefPosition.LEFT : ReefPosition.RIGHT)
                .alongWith(coralSuperstructure.goToSetpointProfiled(() -> setpoint))
                .withDeadline(
                    Commands.waitSeconds(0.5)
                        .andThen(
                            coralSuperstructure
                                .outtakeCoral()
                                // .until(() -> !coralSuperstructure.hasCoral())
                                .withTimeout(0.5))));
  }

  private Command toPathCommand(PathPlannerPath path, boolean zero) {
    if (path == null) return Commands.none();
    Pose2d startingPose =
        new Pose2d(path.getPoint(0).position, path.getIdealStartingState().rotation());
    return zero
        ? AutoBuilder.resetOdom(startingPose).andThen(AutoBuilder.followPath(path))
        : AutoBuilder.followPath(path);
  }

  private Command toPathCommand(PathPlannerPath path) {
    return toPathCommand(path, false);
  }

  private PathPlannerPath getPath(String pathName)
      throws FileVersionException, IOException, ParseException {
    // Load the path you want to follow using its name in the GUI
    return PathPlannerPath.fromPathFile(pathName);
  }

  enum StartingPosition {
    TOP("Starting 1"),
    MIDDLE("Starting 2"),
    BOTTOM("Starting 3");

    private String pathID;

    StartingPosition(String pathID) {
      this.pathID = pathID;
    }
  }

  enum ReefSide {
    NOCHOICE("Brake"),
    REEFR1("ReefR1"),
    REEFR2("ReefR2"),
    REEFR3("ReefR3"),
    REEFL1("ReefL1"),
    REEFL2("ReefL2"),
    REEFL3("ReefL3");

    private String pathID;

    ReefSide(String pathID) {
      this.pathID = pathID;
    }
  }

  enum Level {
    NOCHOICE,
    L1,
    L2,
    L3,
    L4;
  }

  enum Pole {
    NOCHOICE,
    LEFTPOLE,
    RIGHTPOLE;
  }

  enum FeedLocation {
    NOCHOICE("Brake"),
    UPCORALRIGHT("UpCoralRight"),
    UPCORALMIDDLE("UpCoralMiddle"),
    UPCORALLEFT("UpCoralLeft"),
    DOWNCORALRIGHT("DownCoralRight"),
    DOWNCORALMIDDLE("DownCoralMiddle"),
    DOWNCORALLEFT("DownCoralLeft");

    private String pathID = "";

    FeedLocation(String pathID) {
      this.pathID = pathID;
    }
  }

  enum CoralSide {
    LEFT,
    MIDDLE,
    RIGHT;
  }

  enum PreBuiltAuto {
    TOPAUTO,
    TAXI,
    MIDTOPAUTO,
    MIDBOTAUTO,
    BOTAUTO,
    CUSTOM,
    MIDPRELOADAUTO,
    DO_NOTHING,
    TEST,
    MIDOPPOSITESIDEAUTO;
  }

  public static class ScoringGroupChooser {
    // Adds sendable choosers
    private SendableChooser<ReefSide> reefSide = new SendableChooser<>();
    private SendableChooser<Level> level = new SendableChooser<>();
    private SendableChooser<Pole> pole = new SendableChooser<>();
    private SendableChooser<FeedLocation> feedLocation = new SendableChooser<>();
    private SendableChooser<CoralSide> coralSide = new SendableChooser<>();

    // add left middle right options for downcoral and upcoral, align to middle in pathplanner, 1.5
    // speed going to coral

    public ScoringGroupChooser(int index) {

      reefSide.setDefaultOption("No Choice", ReefSide.NOCHOICE);
      reefSide.addOption("ReefR1", ReefSide.REEFR1);
      reefSide.addOption("ReefR2", ReefSide.REEFR2);
      reefSide.addOption("ReefR3", ReefSide.REEFR3);
      reefSide.addOption("ReefL1", ReefSide.REEFL1);
      reefSide.addOption("ReefL2", ReefSide.REEFL2);
      reefSide.addOption("ReefL3", ReefSide.REEFL3);

      level.setDefaultOption("No Choice", Level.NOCHOICE);
      level.addOption("L1", Level.L1);
      level.addOption("L2", Level.L2);
      level.addOption("L3", Level.L3);
      level.addOption("L4", Level.L4);

      pole.setDefaultOption("Right", Pole.RIGHTPOLE);
      pole.addOption("Left", Pole.LEFTPOLE);

      feedLocation.setDefaultOption("No Choice", FeedLocation.NOCHOICE);
      feedLocation.addOption("Down Coral Left", FeedLocation.DOWNCORALLEFT);
      feedLocation.addOption("Down Coral Middle", FeedLocation.DOWNCORALMIDDLE);
      feedLocation.addOption("Down Coral Right", FeedLocation.DOWNCORALRIGHT);
      feedLocation.addOption("Up Coral Left", FeedLocation.UPCORALLEFT);
      feedLocation.addOption("Up Coral Middle", FeedLocation.UPCORALMIDDLE);
      feedLocation.addOption("Up Coral Right", FeedLocation.UPCORALRIGHT);

      SmartDashboard.putData("Autos/Reef Side" + index, reefSide);
      SmartDashboard.putData("Autos/Level" + index, level);
      SmartDashboard.putData("Autos/Pole" + index, pole);
      SmartDashboard.putData("Autos/FeedLocation" + index, feedLocation);
    }

    public ScoringGroup build() {
      return new ScoringGroup(
          feedLocation.getSelected(),
          reefSide.getSelected(),
          pole.getSelected(),
          level.getSelected());
    }
  }

  public static class ScoringGroup {
    private FeedLocation feedLocation;
    private ReefSide reefSide;
    private Pole pole;
    private Level level;

    public ScoringGroup(FeedLocation feedLocation, ReefSide reefSide, Pole pole, Level level) {
      this.feedLocation = feedLocation;
      this.reefSide = reefSide;
      this.pole = pole;
      this.level = level;
    }
  }

  public static class CycleAutoConfig {
    private List<ScoringGroup> scoringGroup = new ArrayList<>();
    private StartingPosition startingPosition;

    public CycleAutoConfig(StartingPosition startingPosition, List<ScoringGroup> scoringGroup) {
      this.scoringGroup = scoringGroup;
      this.startingPosition = startingPosition;
    }
  }

  public static class CycleAutoChooser {
    private SendableChooser<StartingPosition> startingPosition = new SendableChooser<>();
    private List<ScoringGroupChooser> sgChoosers = new ArrayList<>();

    public CycleAutoChooser(int chooserSize) {

      startingPosition.setDefaultOption("Top", StartingPosition.TOP);
      startingPosition.addOption("Middle", StartingPosition.MIDDLE);
      startingPosition.addOption("Bottom", StartingPosition.BOTTOM);

      SmartDashboard.putData("Autos/Starting Position", startingPosition);

      for (int i = 0; i < chooserSize; i++) sgChoosers.add(new ScoringGroupChooser(i));
    }

    public CycleAutoConfig build() {
      return new CycleAutoConfig(
          startingPosition.getSelected(), sgChoosers.stream().map(a -> a.build()).toList());
    }
  }

  public class PathsAndAuto {
    Command auto;
    List<PathPlannerPath> paths;

    public PathsAndAuto(Command auto, List<PathPlannerPath> paths) {
      this.auto = auto;
      this.paths = paths;
    }

    public Command getAuto() {
      return auto;
    }

    public List<PathPlannerPath> getPaths() {
      return paths;
    }
  }
}
