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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.ArrayList;

/**
 * The Dashboard subsystem manages a collection of dashboard tabs displayed in the Shuffleboard user
 * interface. Other subsystems of the robot can add their own dashboard tab by calling
 * DashboardSubsystem.add(). After all tabs have been added in this manner,
 * DashboardSubsystem.initialize() must be called to call the initDashboard() method of all tabs
 * added to the subsystem. Subsequently, all tabs managed by the DashboardSubsystem have their
 * updateDashboard() method called when the DashboardSubsystem is processed by the scheduler. If
 * running in simulation mode, all tabs have their updateSimulationDashboard() method called
 * instead.
 */
public class DashboardSubsystem extends SubsystemBase {

  private final ArrayList<IDashboardTab> m_dashboardTabs;
  private RobotContainer m_botContainer;

  /** Creates a new Dashboard. */
  public DashboardSubsystem() {
    m_dashboardTabs = new ArrayList<IDashboardTab>();
  }

  /**
   * Adds a dashboard tab to be managed by the subsystem
   *
   * @param tab The dashboard tab to add
   */
  public void add(IDashboardTab tab) {
    m_dashboardTabs.add(tab);
  }

  /**
   * Calling this method calls initDashboard() on each dashboard tab managed by the subsystem
   *
   * @param botContainer A reference to the global RobotContainer instance
   */
  public void initialize(RobotContainer botContainer) {
    m_botContainer = botContainer;
    for (IDashboardTab tab : m_dashboardTabs) {
      tab.initDashboard(botContainer);
    }
  }

  /** Called by the scheduler during each processing cycle to service all dashboard tabs */
  @Override
  public void periodic() {
    for (IDashboardTab tab : m_dashboardTabs) {
      tab.updateDashboard(m_botContainer);
    }
  }

  /**
   * Called by the scheduler during each processing cycle in simulation mode to service all
   * dashboard tabs
   */
  @Override
  public void simulationPeriodic() {
    for (IDashboardTab tab : m_dashboardTabs) {
      tab.updateSimulationDashboard(m_botContainer);
    }
  }
}
