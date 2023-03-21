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
import frc.lib.dashboard.WidgetsWithChangeDetection.ChooserWithChangeDetection;
import frc.lib.dashboard.WidgetsWithChangeDetection.PIDTunerPanel;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.ChargingStation;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.InitialAction;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.DriveToWaypoint;
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
  /** Initial action chooser */
  private final ChooserWithChangeDetection<InitialAction> m_initialActionChooser =
      new ChooserWithChangeDetection<InitialAction>();
  /** Route chooser */
  private final ChooserWithChangeDetection<EscapeRoute.Route> m_routeChooser =
      new ChooserWithChangeDetection<EscapeRoute.Route>();
  /** Secondary action chooser */
  private final ChooserWithChangeDetection<SecondaryAction> m_secondaryActionChooser =
      new ChooserWithChangeDetection<SecondaryAction>();
  /* Balance Position Chooser */
  private final ChooserWithChangeDetection<ChargingStation.BalancePosition>
      m_balancePositionChooser = new ChooserWithChangeDetection<ChargingStation.BalancePosition>();
  /** Waypoint chooser */
  // private final ChooserWithChangeDetection<Waypoints.ID> m_targetWaypointChooser =
  //     new ChooserWithChangeDetection<Waypoints.ID>();

  /** Panel used to set translation PID gains */
  private PIDTunerPanel m_translationPIDPanel;

  /** Panel used to set rotation PID gains */
  private PIDTunerPanel m_rotationPIDPanel;

  /** Used to detect when the present alliance changes */
  private Alliance m_lastAlliance;

  /** Used to maintain a set of displayed Trajectories */
  private HashMap<String, PathPlannerTrajectory> m_fieldTrajectories = new HashMap<>();

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
    m_initialPositionChooser.loadOptions(
        Grids.ScoringPosition.getNames(),
        Grids.ScoringPosition.values(),
        Grids.ScoringPosition.H.id);
    m_tab
        .add("Initial Position", m_initialPositionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0 * kChooserWidth, 0);

    m_initialActionChooser.loadOptions(
        InitialAction.getNames(), InitialAction.values(), InitialAction.ShootHigh.id);
    m_tab
        .add("Initial Action", m_initialActionChooser)
        .withSize(6, kChooserHeight)
        .withPosition(4, 0);

    // Set up a chooser for the route to follow out of the community
    m_routeChooser.loadOptions(EscapeRoute.Route.getNames(), EscapeRoute.Route.values(), 0);
    m_tab.add("Route", m_routeChooser).withSize(8, kChooserHeight).withPosition(10, 0);

    // Set up a chooser for the secondary action to take
    m_secondaryActionChooser.loadOptions(SecondaryAction.getNames(), SecondaryAction.values(), 0);
    m_tab
        .add("Secondary Action", m_secondaryActionChooser)
        .withSize(7, kChooserHeight)
        .withPosition(18, 0);

    // Set up a chooser for the balance position
    m_balancePositionChooser.loadOptions(
        ChargingStation.BalancePosition.getNames(),
        ChargingStation.BalancePosition.values(),
        ChargingStation.BalancePosition.CenterOfCS.id);
    m_tab
        .add("Balance Position", m_balancePositionChooser)
        .withSize(6, kChooserHeight)
        .withPosition(25, 0);

    // Set up a chooser for the waypoint to move to outside the community
    // populateChooser(m_targetWaypointChooser, Waypoints.ID.getNames(), Waypoints.ID.values());
    // m_tab
    //     .add("Waypoint", m_targetWaypointChooser)
    //     .withSize(kChooserWidth, kChooserHeight)
    //     .withPosition(4 * kChooserWidth, 0);

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(24, 14)
        .withPosition(0, 4)
        .withProperties(Map.of("Label position", "HIDDEN"));

    m_translationPIDPanel =
        new PIDTunerPanel(m_tab, "Translation PID", 0, 33, DriveToWaypoint.kDefaultPositionGains);

    m_rotationPIDPanel =
        new PIDTunerPanel(m_tab, "Rotation PID", 8, 33, DriveToWaypoint.kDefaultRotationGains);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    Alliance currentAlliance = DriverStation.getAlliance();

    boolean initialPositionChanged = m_initialPositionChooser.hasChanged();
    boolean initialActionChanged = m_initialActionChooser.hasChanged();
    boolean routeHasChanged = m_routeChooser.hasChanged();
    boolean selectedActionChanged = m_secondaryActionChooser.hasChanged();
    boolean balancePositionChanged = m_balancePositionChooser.hasChanged();
    boolean targetWaypointChanged = false; // m_targetWaypointChooser.hasChanged();
    boolean translationPIDChanged = m_translationPIDPanel.hasChanged();
    boolean rotationPIDChanged = m_rotationPIDPanel.hasChanged();

    if ((m_lastAlliance != currentAlliance)
        || initialPositionChanged
        || initialActionChanged
        || routeHasChanged
        || selectedActionChanged
        || balancePositionChanged
        || targetWaypointChanged
        || translationPIDChanged
        || rotationPIDChanged) {

      System.out.println("<AutoDashboardTab::updateDashboard> processing dashboard value change");
      m_lastAlliance = currentAlliance;

      // Rebuild the auto routine
      m_builder.build(
          botContainer,
          m_initialPositionChooser.getSelected(),
          m_initialActionChooser.getSelected(),
          m_routeChooser.getSelected(),
          m_secondaryActionChooser.getSelected(),
          m_balancePositionChooser.getSelected(),
          m_translationPIDPanel.getGains(),
          m_rotationPIDPanel.getGains(),
          false);

      // Set all objects on the field to have an empty trajectory
      for (String name : m_fieldTrajectories.keySet()) {
        m_fieldTrajectories.put(name, new PathPlannerTrajectory());
      }

      // Get trajectories for the active auto
      List<PathPlannerTrajectory> trajectories = m_builder.getTrajectories();

      // Add them to the map of field objects
      for (int idx = 0; idx < trajectories.size(); ++idx) {
        String name = String.format("AutoTrajectory%d", idx);
        PathPlannerTrajectory traj = trajectories.get(idx);
        m_fieldTrajectories.put(name, traj);
      }

      // Send all trajectories to the field
      m_fieldTrajectories.forEach(
          (String name, PathPlannerTrajectory traj) ->
              m_field2d.getObject(name).setTrajectory(traj));

      Pose2d pose = m_initialPositionChooser.getSelected().getPose();
      botContainer.swerveSubsystem.resetOdometry(pose);
    }

    m_field2d.setRobotPose(botContainer.swerveSubsystem.getPose());
  }

  /** Returns the current auto routine builder */
  public AutoRoutineBuilder getAutoBuilder() {
    return m_builder;
  }

  /** Returns the field displayed in the dashboard tab */
  public Field2d getField2d() {
    return m_field2d;
  }
}
