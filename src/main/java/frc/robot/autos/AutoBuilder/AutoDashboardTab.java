////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5920 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/
package frc.robot.autos.AutoBuilder;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.utility.PIDTunerPanel;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import java.util.*;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class AutoDashboardTab implements IDashboardTab {

  /** Title displayed in the dashboard tab */
  static final String kTabTitle = "Auto Builder";

  /** Width (in cells) of the field display */
  static final int kFieldWidthCells = 20;

  /** Height (in cells) of the field display */
  static final int kFieldHeightCells = 11;

  /** Width (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserWidth = 4;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserHeight = 2;

  /** Builder used to generate Auto commands */
  private AutoRoutineBuilder m_builder;

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Initial position chooser */
  private final ChooserWithChangeDetection<Grids.ScoringPosition> m_initialPositionChooser =
      new ChooserWithChangeDetection<Grids.ScoringPosition>();
  /** Route chooser */
  private final ChooserWithChangeDetection<EscapeRoute.Route> m_routeChooser =
      new ChooserWithChangeDetection<EscapeRoute.Route>();
  /** Secondary action chooser */
  private final ChooserWithChangeDetection<SecondaryAction> m_secondaryActionChooser =
      new ChooserWithChangeDetection<SecondaryAction>();
  /** Waypoint chooser */
  // private final ChooserWithChangeDetection<Waypoints.ID> m_targetWaypointChooser =
  //     new ChooserWithChangeDetection<Waypoints.ID>();

  /** Panel used to set translation PID gains */
  private PIDTunerPanel m_translationPIDPanel;

  /** Panel used to set rotation PID gains */
  private PIDTunerPanel m_rotationPIDPanel;

  private Alliance m_lastAlliance;

  /** Creates an instance of the tab */
  public AutoDashboardTab(AutoRoutineBuilder autoBuilder) {
    m_builder = autoBuilder;
    m_field2d = new Field2d();
  }

  /**
   * Create and initialize dashboard widgets
   *
   * @param botContainer Container that holds robot subsystems
   */
  @Override
  public void initDashboard(RobotContainer botContainer) {
    m_tab = Shuffleboard.getTab(kTabTitle);
    m_lastAlliance = DriverStation.getAlliance();

    // Set up the initial position chooser
    populateChooser(
        m_initialPositionChooser, Grids.ScoringPosition.getNames(), Grids.ScoringPosition.values());
    m_tab
        .add("Initial Position", m_initialPositionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0 * kChooserWidth, 0);

    // Set up a chooser for the route to follow out of the community
    populateChooser(m_routeChooser, EscapeRoute.Route.getNames(), EscapeRoute.Route.values());
    m_tab
        .add("Route", m_routeChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(1 * kChooserWidth, 0);

    // Set up a chooser for the secondary action to take
    populateChooser(m_secondaryActionChooser, SecondaryAction.getNames(), SecondaryAction.values());
    m_tab
        .add("Secondary Action", m_secondaryActionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(3 * kChooserWidth, 0);

    // Set up a chooser for the waypoint to move to outside the community
    // populateChooser(m_targetWaypointChooser, Waypoints.ID.getNames(), Waypoints.ID.values());
    // m_tab
    //     .add("Waypoint", m_targetWaypointChooser)
    //     .withSize(kChooserWidth, kChooserHeight)
    //     .withPosition(4 * kChooserWidth, 0);

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(kFieldWidthCells, kFieldHeightCells)
        .withPosition(0, kChooserHeight + 1)
        .withProperties(Map.of("Label position", "HIDDEN"));

    m_translationPIDPanel =
        new PIDTunerPanel(
            m_tab, "Translation PID", 0, kFieldWidthCells, DriveToWaypoint.kDefaultPositionGains);
    // new frc.lib.utility.PIDGains(
    //     AutoRoutineBuilder.kDefaultTranslationkP,
    //     AutoRoutineBuilder.kDefaultTranslationkI,
    //     AutoRoutineBuilder.kDefaultTranslationkD));

    m_rotationPIDPanel =
        new PIDTunerPanel(
            m_tab, "Rotation PID", 0, kFieldWidthCells, DriveToWaypoint.kDefaultRotationGains);
    // new frc.lib.utility.PIDGains(
    //     AutoRoutineBuilder.kDefaultRotationkP,
    //     AutoRoutineBuilder.kDefaultRotationkI,
    //     AutoRoutineBuilder.kDefaultRotationkD));
  }

  /**
   * Populates a String chooser
   *
   * @param chooser The chooser to populate
   * @param choices An array of strings representing the choices
   * @post The chooser will be populated with the given choices and the default value will be set to
   *     the first choice
   */
  private static <V> void populateChooser(
      SendableChooser<V> chooser, String choices[], V values[]) {
    for (int i = 0; i < choices.length; ++i) {
      String choice = choices[i];
      V value = values[i];
      chooser.addOption(choice, value);
    }

    chooser.setDefaultOption(choices[0], values[0]);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    Alliance currentAlliance = DriverStation.getAlliance();

    boolean initialPositionChanged = m_initialPositionChooser.hasChanged();
    boolean routeHasChanged = m_routeChooser.hasChanged();
    boolean selectedActionChanged = m_secondaryActionChooser.hasChanged();
    boolean targetWaypointChanged = false; // m_targetWaypointChooser.hasChanged();
    boolean translationPIDChanged = m_translationPIDPanel.hasChanged();
    boolean rotationPIDChanged = m_rotationPIDPanel.hasChanged();

    if ((m_lastAlliance != currentAlliance)
        || initialPositionChanged
        || routeHasChanged
        || selectedActionChanged
        || targetWaypointChanged
        || translationPIDChanged
        || rotationPIDChanged) {

      m_lastAlliance = currentAlliance;

      // Rebuild the auto routine
      m_builder.build(
          botContainer,
          m_initialPositionChooser.getSelected(),
          m_routeChooser.getSelected(),
          m_secondaryActionChooser.getSelected(),
          m_translationPIDPanel.getGains(),
          m_rotationPIDPanel.getGains());

      // Display auto trajectories on the field
      List<PathPlannerTrajectory> trajectories = m_builder.getTrajectories();
      for (int idx = 0; idx < trajectories.size(); ++idx) {
        m_field2d
            .getObject(String.format("AutoTrajectory%d", idx))
            .setTrajectory(trajectories.get(idx));
      }

      Pose2d pose = m_initialPositionChooser.getSelected().getPose();
      botContainer.swerveSubsystem.resetOdometry(pose);
      SmartDashboard.putNumber("initialX", pose.getX());
      SmartDashboard.putNumber("initialY", pose.getY());
    }

    m_field2d.setRobotPose(botContainer.swerveSubsystem.getPose());
  }

  /** Returns the current auto routine builder */
  public AutoRoutineBuilder getAutoBuilder() {
    return m_builder;
  }

  private static class ChooserWithChangeDetection<V> extends SendableChooser<V> {
    private V m_lastValue;
    private boolean m_initialChangeFlag = true;

    public boolean hasChanged() {
      boolean changed = m_initialChangeFlag || (m_lastValue != getSelected());
      m_initialChangeFlag = false;
      m_lastValue = getSelected();
      return changed;
    }
  }
}
