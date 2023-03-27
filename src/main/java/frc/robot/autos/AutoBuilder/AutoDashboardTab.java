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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.dashboard.WidgetsWithChangeDetection.ChooserWithChangeDetection;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.autos.AutoConstants.ChargingStation;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.InitialAction;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.Preset.LinkAndBalanceAutoBuilder;
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
  static final int kChooserWidth = 7;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserHeight = 1;

  /** Builder used to generate Auto commands */
  private AutoRoutineBuilder m_autoBuilder;

  /** Builder used to generate Link+Balance auto commands */
  private LinkAndBalanceAutoBuilder m_linkAndBalanceBuilder;

  /** The current selected auto command */
  private CommandBase m_currentAutoRoutine;

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /* Auto type Chooser used to select between generated and preset autos */
  private final ChooserWithChangeDetection<AutoType> m_autoTypeChooser =
      new ChooserWithChangeDetection<AutoType>();

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

  /** Used to detect when the present alliance changes */
  private Alliance m_lastAlliance;

  /** Used to maintain a set of displayed Trajectories */
  private HashMap<String, PathPlannerTrajectory> m_fieldTrajectories = new HashMap<>();

  /** Creates an instance of the tab */
  public AutoDashboardTab() {
    m_autoBuilder = new AutoRoutineBuilder();
    m_linkAndBalanceBuilder = new LinkAndBalanceAutoBuilder();
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

    // Set up the auto type chooser
    m_autoTypeChooser.loadOptions(AutoType.getNames(), AutoType.values(), AutoType.AutoBuilder.id);
    m_tab
        .add("Auto Type", m_autoTypeChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(24, 0)
        .withProperties(Map.of("Label position", "LEFT"));

    setupAutoBuilderLayout(m_tab);

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(24, 14)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));
  }

  private void setupAutoBuilderLayout(ShuffleboardTab tab) {
    ShuffleboardLayout autoBuilderLayout =
        tab.getLayout("AutoBuilder Config", BuiltInLayouts.kGrid)
            .withSize(kChooserWidth, 5)
            .withPosition(24, 3)
            .withProperties(
                Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 5));

    // Set up the initial position chooser
    m_initialPositionChooser.loadOptions(
        Grids.ScoringPosition.getNames(),
        Grids.ScoringPosition.values(),
        Grids.ScoringPosition.H.id);
    autoBuilderLayout
        .add("Initial Position", m_initialPositionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0, 0);

    m_initialActionChooser.loadOptions(
        InitialAction.getNames(), InitialAction.values(), InitialAction.ShootHigh.id);
    autoBuilderLayout
        .add("Initial Action", m_initialActionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0, 1);

    // Set up a chooser for the route to follow out of the community
    m_routeChooser.loadOptions(EscapeRoute.Route.getNames(), EscapeRoute.Route.values(), 0);
    autoBuilderLayout
        .add("Route", m_routeChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0, 2);

    // Set up a chooser for the secondary action to take
    m_secondaryActionChooser.loadOptions(SecondaryAction.getNames(), SecondaryAction.values(), 0);
    autoBuilderLayout
        .add("Secondary Action", m_secondaryActionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0, 3);

    // Set up a chooser for the balance position
    m_balancePositionChooser.loadOptions(
        ChargingStation.BalancePosition.getNames(),
        ChargingStation.BalancePosition.values(),
        ChargingStation.BalancePosition.CenterOfCS.id);
    autoBuilderLayout
        .add("Balance Position", m_balancePositionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0, 4);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    Alliance currentAlliance = DriverStation.getAlliance();

    boolean autoTypeChanged = m_autoTypeChooser.hasChanged();
    boolean initialPositionChanged = m_initialPositionChooser.hasChanged();
    boolean initialActionChanged = m_initialActionChooser.hasChanged();
    boolean routeHasChanged = m_routeChooser.hasChanged();
    boolean selectedActionChanged = m_secondaryActionChooser.hasChanged();
    boolean balancePositionChanged = m_balancePositionChooser.hasChanged();

    if ((m_lastAlliance != currentAlliance)
        || autoTypeChanged
        || initialPositionChanged
        || initialActionChanged
        || routeHasChanged
        || selectedActionChanged
        || balancePositionChanged) {

      System.out.println("<AutoDashboardTab::updateDashboard> processing dashboard value change");
      m_lastAlliance = currentAlliance;

      // Clear trajectories displayed on the field by giving all field objects an empty trajectory
      PathPlannerTrajectory emptyTrajectory = new PathPlannerTrajectory();
      for (String name : m_fieldTrajectories.keySet()) {
        m_fieldTrajectories.put(name, emptyTrajectory);
      }

      Pose2d initialPose = null;
      List<PathPlannerTrajectory> trajectories = null;

      // Set up the new auto routine
      switch (m_autoTypeChooser.getSelected()) {
        case AutoBuilder:
          {
            // Rebuild the auto routine
            m_currentAutoRoutine =
                m_autoBuilder.build(
                    botContainer,
                    m_initialPositionChooser.getSelected(),
                    m_initialActionChooser.getSelected(),
                    m_routeChooser.getSelected(),
                    m_secondaryActionChooser.getSelected(),
                    m_balancePositionChooser.getSelected(),
                    EscapeStrategy.kDefaultMotionConfig,
                    BalanceStrategy.kDefaultMotionConfig);

            // Get trajectories for the active auto
            trajectories = m_autoBuilder.getTrajectories();
            initialPose = m_initialPositionChooser.getSelected().getPose();
            break;
          }

        case LinkNBalance:
          {
            trajectories = m_linkAndBalanceBuilder.getTrajectories();
            initialPose = m_linkAndBalanceBuilder.getInitialPose();
            m_currentAutoRoutine =
                m_linkAndBalanceBuilder.getCommand(AutoType.LinkNBalance, botContainer);
          }
      }

      // Add trajectories to the map of field objects
      for (int idx = 0; idx < trajectories.size(); ++idx) {
        String name = String.format("AutoTrajectory%d", idx);
        PathPlannerTrajectory traj = trajectories.get(idx);
        m_fieldTrajectories.put(name, traj);
      }

      // Send all trajectories to the field
      m_fieldTrajectories.forEach(
          (String name, PathPlannerTrajectory traj) ->
              m_field2d.getObject(name).setTrajectory(traj));

      botContainer.swerveSubsystem.resetOdometry(initialPose);
    }

    m_field2d.setRobotPose(botContainer.swerveSubsystem.getPose());
  }

  /** Returns the current auto routine builder */
  public CommandBase getCurrentAutoRoutine() {
    return m_currentAutoRoutine;
  }

  /** Returns the field displayed in the dashboard tab */
  public Field2d getField2d() {
    return m_field2d;
  }
}
