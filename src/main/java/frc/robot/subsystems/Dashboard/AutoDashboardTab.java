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
package frc.robot.subsystems.Dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoConstants.CargoLocation;
import frc.robot.AutoConstants.StagingLocation;
import frc.robot.AutoConstants.Substation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
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
  static final int kSubPanelWidth = 6;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kSubPanelHeight = 6;

  static final String kCargoNames[] = CargoLocation.getNames();

  static final String kGoalNames[] = Substation.getNames();

  static final String kStagingLocationNames[] = StagingLocation.getNames();

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Initial position chooser */
  private final SendableChooser<String> m_initialPositionChooser = new SendableChooser<>();
  /** Staging position/route chooser */
  private final SendableChooser<String> m_stagingRouteChooser = new SendableChooser<>();
  /** Cargo position chooser */
  private final SendableChooser<String> m_cargoPosition = new SendableChooser<>();

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
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));

    // Set up the initial position chooser
    for (int i = 0; i < kGoalNames.length; ++i) {
      String goalName = kGoalNames[i];
      if (0 == i) {
        m_initialPositionChooser.setDefaultOption(goalName, goalName);
      } else {
        m_initialPositionChooser.addOption(goalName, goalName);
      }
    }
    m_tab
        .add("Initial Position", m_initialPositionChooser)
        .withSize(3, 1)
        .withPosition(kFieldHeightCells, 0);

    // Set up the lane chooser
    for (int i = 0; i < kGoalNames.length; ++i) {
      String goalName = kGoalNames[i];
      if (0 == i) {
        m_initialPositionChooser.setDefaultOption(goalName, goalName);
      } else {
        m_initialPositionChooser.addOption(goalName, goalName);
      }
    }
    m_tab
        .add("Initial Position", m_initialPositionChooser)
        .withSize(3, 1)
        .withPosition(kFieldHeightCells, 0);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    Swerve swerveSubsystem = botContainer.swerveSubsystem;
    m_field2d.setRobotPose(swerveSubsystem.getPose());
  }
}
