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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.Lane;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.AutoConstants.StagingLocation;
import frc.robot.autos.AutoConstants.Substation;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import java.util.*;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class AutoDashboardTab implements IDashboardTab {

  /** Title displayed in the dashboard tab */
  static final String kTabTitle = "Auto Builder";

  /** Width (in cells) of the field display */
  static final int kFieldWidthCells = 10;

  /** Height (in cells) of the field display */
  static final int kFieldHeightCells = 6;

  /** Width (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserWidth = 4;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kChooserHeight = 1;

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Initial position chooser */
  private final ChooserWithChangeDetection<Substation> m_initialPositionChooser =
      new ChooserWithChangeDetection<Substation>();
  /** Staging position/route chooser */
  private final ChooserWithChangeDetection<Lane> m_laneChooser =
      new ChooserWithChangeDetection<Lane>();
  /** Staging position/route chooser */
  private final ChooserWithChangeDetection<StagingLocation> m_stagingRouteChooser =
      new ChooserWithChangeDetection<StagingLocation>();
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
    populateChooser(m_initialPositionChooser, Substation.getNames(), Substation.values());
    m_tab
        .add("Initial Position", m_initialPositionChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(0 * kChooserWidth, kFieldHeightCells);

    // Set up the lane chooser
    populateChooser(m_laneChooser, Lane.getNames(), Lane.values());
    m_tab
        .add("Lane", m_laneChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(1 * kChooserWidth, kFieldHeightCells);

    // Set up a chooser for the staging/transitional location to pass through
    populateChooser(m_stagingRouteChooser, StagingLocation.getNames(), StagingLocation.values());
    m_tab
        .add("Route", m_stagingRouteChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(2 * kChooserWidth, kFieldHeightCells);

    // Set up a chooser for the secondary action to take
    populateChooser(m_secondaryActionChooser, SecondaryAction.getNames(), SecondaryAction.values());
    m_tab
        .add("Secondary Action", m_stagingRouteChooser)
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(3 * kChooserWidth, kFieldHeightCells);
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
      if (0 == i) {
        chooser.setDefaultOption(choice, value);
      } else {
        chooser.addOption(choice, value);
      }
    }
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    if (m_initialPositionChooser.hasChanged()
        || m_laneChooser.hasChanged()
        || m_stagingRouteChooser.hasChanged()
        || m_secondaryActionChooser.hasChanged()) {
      // Rebuild the auto routine
      m_builder =
          new AutoRoutineBuilder(
              m_initialPositionChooser.getSelected(),
              m_laneChooser.getSelected(),
              m_stagingRouteChooser.getSelected());
    }
  }

  /** Returns the current auto routine builder */
  public AutoRoutineBuilder getAutoBuilder() {
    return m_builder;
  }

  private static class ChooserWithChangeDetection<V> extends SendableChooser<V> {
    private V m_lastValue;

    @Override
    public void setDefaultOption(String name, V object) {
      super.setDefaultOption(name, object);
      m_lastValue = object;
    }

    public boolean hasChanged() {
      return m_lastValue != getSelected();
    }
  }
}
