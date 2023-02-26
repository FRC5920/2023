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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.Lanes;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.AutoConstants.Waypoints;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import java.util.*;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class AutoDashboardTab implements IDashboardTab {

  /** Title displayed in the dashboard tab */
  static final String kTabTitle = "Auto Builder";

  /** Width (in cells) of the field display */
  static final int kFieldWidthCells = 16;

  /** Height (in cells) of the field display */
  static final int kFieldHeightCells = 11;

  /** Width (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserWidth = 4;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserHeight = 1;

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Initial position chooser */
  private final ChooserWithChangeDetection<Grids.ScoringPosition> m_initialPositionChooser =
      new ChooserWithChangeDetection<Grids.ScoringPosition>();
  /** Staging position/route chooser */
  private final ChooserWithChangeDetection<Lanes.Lane> m_laneChooser =
      new ChooserWithChangeDetection<Lanes.Lane>();
  /** Route chooser */
  private final ChooserWithChangeDetection<EscapeRoute.Route> m_routeChooser =
      new ChooserWithChangeDetection<EscapeRoute.Route>();
  /** Waypoint chooser */
  private final ChooserWithChangeDetection<Waypoints.ID> m_escapeWaypointChooser =
      new ChooserWithChangeDetection<Waypoints.ID>();

  /** Cargo position chooser */
  private final ChooserWithChangeDetection<SecondaryAction> m_secondaryActionChooser =
      new ChooserWithChangeDetection<SecondaryAction>();

  private AutoRoutineBuilder m_builder;

  /** Creates an instance of the tab */
  public AutoDashboardTab() {
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

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(kFieldWidthCells, kFieldHeightCells)
        .withPosition(0, 0 * kChooserWidth)
        .withProperties(Map.of("Label position", "HIDDEN"));

    // Set up the initial position chooser
    populateChooser(
        m_initialPositionChooser, Grids.ScoringPosition.getNames(), Grids.ScoringPosition.values());
    m_tab
        .add("Initial Position", m_initialPositionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0 * kChooserWidth, kFieldHeightCells);

    // Set up a chooser for the route to follow out of the community
    populateChooser(m_routeChooser, EscapeRoute.Route.getNames(), EscapeRoute.Route.values());
    m_tab
        .add("Route", m_routeChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(2 * kChooserWidth, kFieldHeightCells);

    // Set up a chooser for the secondary action to take
    populateChooser(m_secondaryActionChooser, SecondaryAction.getNames(), SecondaryAction.values());
    m_tab
        .add("Secondary Action", m_secondaryActionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(4 * kChooserWidth, kFieldHeightCells);

    // Set up a chooser for the waypoint to move to outside the community
    populateChooser(m_escapeWaypointChooser, Waypoints.ID.getNames(), Waypoints.ID.values());
    m_tab
        .add("Waypoint", m_escapeWaypointChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(3 * kChooserWidth, kFieldHeightCells);

    // Create an auto routine builder
    m_builder = new AutoRoutineBuilder(botContainer);
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

    if (m_initialPositionChooser.hasChanged()
        || m_laneChooser.hasChanged()
        || m_routeChooser.hasChanged()
        || m_secondaryActionChooser.hasChanged()) {
      // Rebuild the auto routine

      m_builder.build(
          botContainer,
          m_initialPositionChooser.getSelected(),
          m_routeChooser.getSelected(),
          m_escapeWaypointChooser.getSelected());

      // Display the auto trajectory on the field
      m_field2d.getObject("EscapeTrajectory").setTrajectory(m_builder.getTrajectory());
    }

    if (m_initialPositionChooser.hasChanged()) {
      Pose2d pose = m_initialPositionChooser.getSelected().pose;
      m_field2d.setRobotPose(pose);
      SmartDashboard.putNumber("initialX", pose.getX());
      SmartDashboard.putNumber("initialY", pose.getY());
    }
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
