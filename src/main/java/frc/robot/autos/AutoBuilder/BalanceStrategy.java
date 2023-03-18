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

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.lib.utility.PIDGains;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.ChargingStation.BalancePosition;
import frc.robot.autos.AutoConstants.Waypoints;
import frc.robot.commands.Balance;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class BalanceStrategy {

  /** Maximum velocity when driving to Charging Station */
  public static final double kMaxVelocity = 6.0;

  /** Maximum velocity when driving to Charging Station */
  public static final double kMaxAcceleration = 8.0;

  // Create a trajectory from the waypoints
  private static final PathConstraints kDefaultPathConstraints =
      new PathConstraints(kMaxVelocity, kMaxAcceleration);

  /** A list of trajectories followed for the auto (for display) */
  private List<PathPlannerTrajectory> m_trajectories;

  /** Initial location of the bot */
  PathPointHelper m_initialLocation;

  /** Balance position on the charging station */
  BalancePosition m_balancePosition;

  /**
   * Creates an instance of the strategy
   *
   * @param initialLocation Initial location of the bot when the strategy begins
   */
  BalanceStrategy(PathPointHelper initialLocation, BalancePosition balancePosition) {
    m_initialLocation = initialLocation;
    m_balancePosition = balancePosition;
    generateTrajectories();
  }

  /** Returns trajectories for the auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
  }

  public static void eatPose(Pose2d p) {}

  /** Generates a command sequence to drive to the Charging Station and balance */
  Command generateCommand(
      Swerve swerveSubsystem, PIDGains translationPIDGains, PIDGains rotationPIDGains) {
    // Map of events
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            BalanceStrategy
                ::eatPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            swerveSubsystem.getSwerveKinematics(), // SwerveDriveKinematics
            new PIDConstants(
                translationPIDGains.kP,
                translationPIDGains.kI,
                translationPIDGains
                    .kD), // PID constants to correct for translation error (used to create
            // the X and Y PID controllers)
            new PIDConstants(
                rotationPIDGains.kP,
                rotationPIDGains.kI,
                rotationPIDGains
                    .kD), // PID constants to correct for rotation error (used to create the
            // rotation controller)
            swerveSubsystem
                ::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            false, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path
            // following commands
            );

    SequentialCommandGroup commands = new SequentialCommandGroup();

    commands.addCommands(
        /** Print command execution */
        new PrintCommand(String.format("Drive to charging station and balance")),
        /** Drive to the Charging Station */
        autoBuilder.fullAuto(m_trajectories),
        /** Balance on the Charging Station */
        new Balance(swerveSubsystem));

    commands.addCommands(new PrintCommand("My escape is complete"));
    return commands;
  }

  /** Generates trajectories representing the bot's path to the charging station */
  private void generateTrajectories() {
    ArrayList<PathPlannerTrajectory> trajectoryList = new ArrayList<PathPlannerTrajectory>();
    Translation2d y = Waypoints.ID.Y.getPosition();
    Translation2d cs = m_balancePosition.getBalancePosition();

    // PathPlanner doesn't automatically adjust rotations according to Alliance
    Rotation2d fieldFacing = AllianceFlipUtil.apply(BotOrientation.kFacingField);
    Rotation2d gridFacing = AllianceFlipUtil.apply(BotOrientation.kFacingGrid);

    PathPointHelper initialPoint =
        new PathPointHelper(
            "Initial pose",
            m_initialLocation.getX(),
            m_initialLocation.getY(),
            fieldFacing, // Heading needs to be facing field to get the right spline
            gridFacing);
    PathPointHelper stageAtY =
        new PathPointHelper(
            "Stage at Y",
            y.getX(),
            m_balancePosition.getBalancePosition().getY(),
            gridFacing,
            gridFacing);
    PathPointHelper centerOfCS =
        new PathPointHelper("Center of CS", cs.getX(), cs.getY(), gridFacing, gridFacing);

    trajectoryList.add(
        PathPlanner.generatePath(kDefaultPathConstraints, initialPoint, stageAtY, centerOfCS));

    m_trajectories = trajectoryList;
  }
}
