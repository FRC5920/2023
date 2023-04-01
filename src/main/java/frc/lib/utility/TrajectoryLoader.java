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
package frc.lib.utility;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.lib.utility.BotLog.MessageType;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.HashMap;

/** A helper class used to store settings and load a Trajectory file */
public class TrajectoryLoader {
  /** File name to load from */
  private final String m_trajectoryFileName;

  /** Path constraints applied when following the trajectory */
  private final PathConstraints m_pathConstraints;

  /** Trajectory loaded from file */
  private PathPlannerTrajectory m_trajectory;

  /**
   * Creates a TrajectoryLoader that will load a given pathplanner file and apply given constraints
   *
   * @param trajectoryFileName Name of the trajectory file to load
   * @param maxVelocity Max velocity applied to the loaded trajectory
   * @param maxAcceleration Max acceleration applied to the loaded trajectory
   * @note In the source directory, PathPlanner (.path) files are located under the directory
   *     "src\deploy\pathplanner". These files are automatically copied to a corresponding deploy
   *     directory on the RIO when code is downloaded to the robot. The PathPlanner.loadPath()
   *     function used to load trajectories from these files automatically resolves the
   *     deploy/pathplanner directory on the RIO at runtime and appends a ".path" extension to the
   *     trajectory file name it is passed. The TrajectoryLoader class automatically strips off any
   *     ".path" file extension present in a file name when calling PathPlanner.loadPath().
   */
  public TrajectoryLoader(String trajectoryFileName, double maxVelocity, double maxAcceleration) {
    m_trajectoryFileName = trajectoryFileName;
    m_pathConstraints = new PathConstraints(maxVelocity, maxAcceleration);
    m_trajectory = loadTrajectory(m_trajectoryFileName, m_pathConstraints);
  }

  /** Returns the initial pose of the object's trajectory */
  public Pose2d getInitialPose() {
    return AllianceFlipUtil.apply(m_trajectory.getInitialHolonomicPose());
  }

  public PathPlannerTrajectory getTrajectory() {
    return m_trajectory;
  }

  /**
   * Generates a command that will follow the object's trajectory
   *
   * @param eventMap Map used to launch commands for events raised in the trajectory
   * @param swerveSubsystem Swerve subsystem used to follow the trajectory
   * @param translationGains PID gains applied to translation when following the trajectory
   * @param rotationGains PID gains applied to holonomic rotation when following the trajectory
   * @param pathConstraints Velocity and acceleration constraints applied when following the
   *     trajectory
   */
  public CommandBase generateTrajectoryCommand(
      String autoName,
      HashMap<String, Command> eventMap,
      Swerve swerveSubsystem,
      PIDConstants translationGains,
      PIDConstants rotationGains,
      PathConstraints pathConstraints) {
    return Commands.sequence(
        new BotLog.PrintCommand(
            MessageType.Debug, autoName + " follow trajectory: " + m_trajectoryFileName),
        buildTrajectoryCommand(
            m_trajectory,
            eventMap,
            swerveSubsystem,
            translationGains,
            rotationGains,
            pathConstraints));
  }

  private static CommandBase buildTrajectoryCommand(
      PathPlannerTrajectory trajectory,
      HashMap<String, Command> eventMap,
      Swerve swerveSubsystem,
      PIDConstants translationGains,
      PIDConstants rotationGains,
      PathConstraints pathConstraints) {
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            // Supplier used to get the bot's present pose
            swerveSubsystem::getPose,
            // Pose2d consumer, used to reset odometry at the beginning of auto
            (p) -> {},
            swerveSubsystem.getSwerveKinematics(), // SwerveDriveKinematics
            // PID gains to correct for translation error
            translationGains,
            // PID gains to correct for rotation error
            rotationGains,
            // Module states consumer used to output to the drive subsystem
            swerveSubsystem::setModuleStates,
            // Map of event names to Commands
            eventMap,
            // true to automatically mirror the path according to alliance color (doesn't work)
            false,
            // The drive subsystem
            swerveSubsystem);

    return autoBuilder.fullAuto(trajectory);
  }

  /**
   * Loads a trajectory using the object's file name and configuration
   *
   * @throws LoadTrajectoryException on failure to load the trajectory file
   */
  private static PathPlannerTrajectory loadTrajectory(
      String trajectoryFileName, PathConstraints pathConstraints) {
    final String dotPath = ".path";
    String filename = trajectoryFileName;

    // Strip ".path" file name extension if present because PathPlanner always appends it
    if (filename.endsWith(dotPath)) {
      filename = filename.substring(0, filename.length() - dotPath.length());
    }

    BotLog.instance.Debug("Load trajectory file: %" + filename);
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(filename, pathConstraints);

    if (trajectory == null) {
      throw new RuntimeException(
          String.format("Failed to load trajectory file: `%s`", trajectoryFileName));
    }

    return trajectory;
  }
}
