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
package frc.robot.autos.Preset;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.AutoType;
import java.util.List;

/** Add your docs here. */
public class PresetAutoBuilder {
  NorthLinkAndBalancePresetBuilder m_northLinkAndBalanceBuilder;
  NorthLinkOverCSPresetBuilder m_northLinkOverCSBuilder;
  SouthLinkAndBalanceBuilder m_southLinkAndBalanceBuilder;

  public PresetAutoBuilder() {
    m_northLinkAndBalanceBuilder = new NorthLinkAndBalancePresetBuilder();
    m_northLinkOverCSBuilder = new NorthLinkOverCSPresetBuilder();
    m_southLinkAndBalanceBuilder = new SouthLinkAndBalanceBuilder();
  }

  public CommandBase getCommand(AutoType autoType, RobotContainer botContainer) {
    CommandBase autoCommand = null;

    switch (autoType) {
      case NorthLinkAndBalance:
        autoCommand = m_northLinkAndBalanceBuilder.getCommand(botContainer);
        break;
      case NorthLinkAndBalanceOverCS:
        autoCommand = m_northLinkOverCSBuilder.getCommand(botContainer);
        break;
      case SouthLinkAndBalance:
        autoCommand = m_southLinkAndBalanceBuilder.getCommand(botContainer);
        break;
      default:
        break; // Return null for unsupported type
    }

    return autoCommand;
  }

  public Pose2d getInitialPose(AutoType autoType) {
    Pose2d pose = null;

    switch (autoType) {
      case NorthLinkAndBalance:
        pose = m_northLinkAndBalanceBuilder.getInitialPose();
        break;
      case NorthLinkAndBalanceOverCS:
        pose = m_northLinkOverCSBuilder.getInitialPose();
        break;
      case SouthLinkAndBalance:
        pose = m_southLinkAndBalanceBuilder.getInitialPose();
        break;
      default:
        break; // Return null for unsupported auto type
    }

    return pose;
  }

  /** Returns a list containing trajectories used to illustrate motion in the auto routine */
  public List<PathPlannerTrajectory> getTrajectories(AutoType autoType) {
    List<PathPlannerTrajectory> trajectories = null;

    switch (autoType) {
      case NorthLinkAndBalance:
        trajectories = m_northLinkAndBalanceBuilder.getTrajectories();
        break;
      case NorthLinkAndBalanceOverCS:
        trajectories = m_northLinkOverCSBuilder.getTrajectories();
        break;
      case SouthLinkAndBalance:
        trajectories = m_southLinkAndBalanceBuilder.getTrajectories();
        break;
      default:
        break; // Return null for unsupported auto type
    }

    return trajectories;
  }
}
