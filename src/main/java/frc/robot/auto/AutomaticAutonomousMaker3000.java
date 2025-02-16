/* (C) Robolancers 2025 */
package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralSuperstructure;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;

public class AutomaticAutonomousMaker3000 {

  private CycleAutoChooser autoChooser = new CycleAutoChooser(3);

  private Field2d field = new Field2d();
  private String pathError = "";
  private List<Pose2d> visualizePath = new ArrayList<>();

  private CoralSuperstructure coralSuperstructure;

  private Command storedAuto;

  public AutomaticAutonomousMaker3000(CoralSuperstructure coralSuperstructure) {
    this.coralSuperstructure = coralSuperstructure;

    SmartDashboard.putData("VisionField", field);
    // Driver has to click submit to make and view the autonomous path
    SmartDashboard.putData(
        "Submit",
        Commands.runOnce(
                () -> {
                  var createdAuto = buildAuto(autoChooser.build());
                  if (createdAuto != null) {
                    storedAuto = createdAuto.getAuto();
                    visualizeAuto(createdAuto.getPaths());
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

    UpdatePathError();
  }

  private void UpdateFieldVisualization() {
    field.getObject("PathPoses").setPoses(visualizePath);
  }

  private void UpdatePathError() {
    SmartDashboard.putString("Path Error", pathError);
  }

  public Command getStoredAuto() {
    return storedAuto;
  }

  private void visualizeAuto(List<PathPlannerPath> paths) {
    visualizePath.clear();

    System.out.println(paths);

    for (int i = 0; i < paths.size(); i++) {
      visualizePath.addAll(paths.get(i).getPathPoses());
    }

    UpdateFieldVisualization();
  }

  // Returns the path list for visualization and autonomous command
  public PathsAndAuto buildAuto(CycleAutoConfig config) {
    pathError = "";
    try {
      Command auto = Commands.none();
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
          auto = auto.andThen(withScoring(toPathCommand(path, "scoring preload")));
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

          auto =
              auto.andThen(withIntaking(toPathCommand(intakePath, "intaking")))
                  .andThen(withScoring(toPathCommand(scorePath, "scoring")));
          lastReefSide = config.scoringGroup.get(i).reefSide;

          paths.add(intakePath);
          paths.add(scorePath);
        }
      }
      return new PathsAndAuto(auto, paths);
    } catch (Exception e) {
      System.out.println(e);
      pathError = "Path doesn't exist";
    }
    return null;
  }

  public Command withIntaking(Command path) {
    return path.alongWith(
        coralSuperstructure.feedCoral().until(() -> coralSuperstructure.hasCoral()));
  }

  public Command withScoring(Command path) {
    // TODO: add scoring logic. Ignore for now as we don't have the code
    return path;
  }

  private Command toPathCommand(PathPlannerPath path, String name) {
    if (path == null) return Commands.none();
    return AutoBuilder.followPath(path)
        .deadlineFor(Commands.run(() -> System.out.println("pathing..." + name)));
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
    UPCORAL("UpCoral"),
    DOWNCORAL("DownCoral");

    private String pathID = "";

    FeedLocation(String pathID) {
      this.pathID = pathID;
    }
  }

  public static class ScoringGroupChooser {
    // Adds sendable choosers
    private SendableChooser<ReefSide> reefSide = new SendableChooser<>();
    private SendableChooser<Level> level = new SendableChooser<>();
    private SendableChooser<Pole> pole = new SendableChooser<>();
    private SendableChooser<FeedLocation> feedLocation = new SendableChooser<>();

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
      feedLocation.addOption("Down Coral", FeedLocation.DOWNCORAL);
      feedLocation.addOption("Up Coral", FeedLocation.UPCORAL);

      SmartDashboard.putData("Reef Side" + index, reefSide);
      SmartDashboard.putData("Level" + index, level);
      SmartDashboard.putData("Pole" + index, pole);
      SmartDashboard.putData("FeedLocation" + index, feedLocation);
    }

    public ScoringGroup build() {
      return new ScoringGroup(feedLocation.getSelected(), reefSide.getSelected());
    }
  }

  public static class ScoringGroup {
    private FeedLocation feedLocation;
    private ReefSide reefSide;

    public ScoringGroup(FeedLocation feedLocation, ReefSide reefSide) {
      this.feedLocation = feedLocation;
      this.reefSide = reefSide;
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

      SmartDashboard.putData("Starting Position", startingPosition);

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
