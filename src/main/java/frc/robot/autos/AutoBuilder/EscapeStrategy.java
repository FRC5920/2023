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
import frc.robot.autos.AutoConstants.BotDimensions;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.DriveToWaypoint;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** The EscapeStrategy class is used to generate auto commands used to escape the community */
public class EscapeStrategy {
  /** Pose where the bot is initially positioned */
  private Grids.ScoringPosition m_startingPosition;

  /** Route the bot will follow to escape from the Community */
  private EscapeRoute.Route m_escapeRoute;

  /** A list of waypoints marking the path of the bot out of the community */
  ArrayList<PathPointHelper> m_waypointList = new ArrayList<PathPointHelper>();

  /** A list of trajectories to follow to escape the community */
  private List<PathPlannerTrajectory> m_escapeTrajectories;

  /** PID gains used for translation during trajectory following */
  private final PIDGains m_translationPIDGains;

  /** PID gains used for rotation during trajectory following */
  private final PIDGains m_rotationPIDGains;

  private final double m_maxVelocity;
  private final double m_maxAcceleration;

  /**
   * Creates an escape strategy using given parameters
   *
   * @param startingPosition Initial position of the bot
   * @param escapeRoute Route the bot should follow to escape the community
   * @param translationPIDGains PID gains used for translation when following trajectories
   * @param rotationPIDGains PID gains used for rotation when following trajectories
   */
  public EscapeStrategy(
      Grids.ScoringPosition startingPosition,
      EscapeRoute.Route escapeRoute,
      PIDGains translationPIDGains,
      PIDGains rotationPIDGains,
      double maxVelocityMetersPerSec,
      double maxAccelerationMetersPerSec2) {
    m_startingPosition = startingPosition;
    m_escapeRoute = escapeRoute;
    m_translationPIDGains = translationPIDGains;
    m_rotationPIDGains = rotationPIDGains;
    m_maxVelocity = maxVelocityMetersPerSec;
    m_maxAcceleration = maxAccelerationMetersPerSec2;

    // Generate waypoints used to carry out the auto
    generatePathWaypoints();

    // Generate trajectories used to carry out the auto
    generateEscapeTrajectories();
  }

  /** Returns a list of trajectories used to carry out the auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    return m_escapeTrajectories;
  }

  /**
   * Generates a command to drive to a series of waypoints to escape the community
   *
   * @param swerveSubsystem Swerve drive subsystem used to drive
   */
  public Command buildWaypointCommandSequence(
      Swerve swerveSubsystem, PIDGains positionPIDGains, PIDGains rotationPIDGains) {

    SequentialCommandGroup commands = new SequentialCommandGroup();

    for (int idx = 1; idx < m_waypointList.size(); ++idx) {
      PathPointHelper waypoint = m_waypointList.get(idx);
      commands.addCommands(
          new PrintCommand(String.format("Drive to waypoint: %s", waypoint.name)),
          new DriveToWaypoint(
              swerveSubsystem,
              new Pose2d(waypoint.getPosition(), waypoint.getHolonomicRotation()),
              DriveToWaypoint.kDefaultPositionToleranceMeters,
              DriveToWaypoint.kDefaultRotationToleranceRadians,
              positionPIDGains,
              rotationPIDGains,
              DriveToWaypoint.kDefaultPositionConstraints,
              DriveToWaypoint.kDefaultRotationConstraints));
    }

    commands.addCommands(new PrintCommand("My escape is complete"));
    return commands;
  }

  /**
   * Generates a command to follow a trajectory to escape the community
   *
   * @param swerveSubsystem Swerve drive subsystem used to drive
   */
  public Command buildTrajectoryCommand(Swerve swerveSubsystem) {

    // Map of events
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem
                ::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            swerveSubsystem.getSwerveKinematics(), // SwerveDriveKinematics
            new PIDConstants(
                m_translationPIDGains.kP,
                m_translationPIDGains.kI,
                m_translationPIDGains
                    .kD), // PID constants to correct for translation error (used to create
            // the X and Y PID controllers)
            new PIDConstants(
                m_rotationPIDGains.kP,
                m_rotationPIDGains.kI,
                m_rotationPIDGains
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

    return autoBuilder.fullAuto(m_escapeTrajectories);
  }

  /**
   * Generates a sequence of waypoints followed to escape the community
   *
   * @postcondition When this method returns, m_waypointList will contain a list of PathhPointHelper
   *     objects corresponding to a sequence of waypoints to travel to escape the community
   */
  private void generatePathWaypoints() {

    Pose2d initialPose = m_startingPosition.getPose();
    Rotation2d initialHolRot = initialPose.getRotation();
    Rotation2d populateLater = new Rotation2d(); // Placeholder for value filled in later
    PathPointHelper initialWaypoint =
        new PathPointHelper(
            "Initial position",
            initialPose.getX(),
            initialPose.getY(),
            populateLater, // Heading
            initialHolRot); // Holonomic rotation

    // Create a PathPoint for the corner of our escape route.
    final EscapeRoute.Corner corner = EscapeRoute.getCorner(m_escapeRoute);
    final Translation2d cornerPosition = corner.getPosition();
    PathPointHelper cornerPoint =
        new PathPointHelper(
            corner.name(),
            cornerPosition.getX(),
            cornerPosition.getY(),
            populateLater, // Heading will get filled in later
            initialHolRot); // Holonomic Rotation ** Maybe change this to
    // BotOrientation.kFacingField for earlier rotation **

    // Generate a waypoint to move into the active lane
    PathPointHelper initialLaneEndpoint =
        new PathPointHelper(
            "initialLaneEndpoint",
            cornerPosition.getX(),
            initialWaypoint.getY(),
            populateLater,
            BotOrientation.kFacingField);

    // Create a waypoint for the endpoint of the active escape route
    final EscapeRoute.Endpoint endpoint = EscapeRoute.getEndpoint(m_escapeRoute);
    PathPointHelper endWaypoint =
        new PathPointHelper(
            endpoint.name(),
            endpoint.getPosition().getX(),
            endpoint.getPosition().getY(),
            BotOrientation.kFacingField,
            AllianceFlipUtil.apply(BotOrientation.kFacingGrid));

    m_waypointList = new ArrayList<>();
    m_waypointList.add(initialWaypoint);
    if (initialLaneEndpoint.distance(initialWaypoint) > BotDimensions.kFootprintWidth) {
      m_waypointList.add(initialLaneEndpoint);
    }
    m_waypointList.add(cornerPoint);
    m_waypointList.add(endWaypoint);

    // DEBUG: Print the list of path waypoints
    // for (PathPointHelper wp : m_waypointList) {
    //   System.out.println(wp.format());
    // }
  }

  /** Generates a path for the bot to follow to get out of the Grid/community */
  private void generateEscapeTrajectories() {
    // Create a trajectory from the waypoints
    PathConstraints escapePathConstraints = new PathConstraints(m_maxVelocity, m_maxAcceleration);

    // Generate a list of trajectories from the waypoint list
    m_escapeTrajectories = generateTrajectoriesFromWaypoints(m_waypointList, escapePathConstraints);
  }

  /** Creates a list of PathPlannerTrajectories from a list of waypoints */
  private static List<PathPlannerTrajectory> generateTrajectoriesFromWaypoints(
      List<PathPointHelper> waypointList, PathConstraints constraints) {
    ArrayList<PathPlannerTrajectory> trajectoryList = new ArrayList<PathPlannerTrajectory>();

    // Move helper points into a list of PathPoints because Java ArrayLists are not polymorphic
    for (int idx = 1; idx < waypointList.size(); ++idx) {
      // Make each waypoint in the list point to the waypoint that follows it
      List<PathPointHelper> pointList =
          alignHeadings(waypointList.get(idx - 1), waypointList.get(idx));
      PathPointHelper start = pointList.get(0);
      PathPointHelper end = pointList.get(1);
      // System.out.printf("Generate trajectory: %s --> %s\n", start.format(), end.format());

      trajectoryList.add(PathPlanner.generatePath(constraints, start, end));
    }

    return trajectoryList;
  }

  private static List<PathPointHelper> alignHeadings(PathPointHelper start, PathPointHelper end) {
    List<PathPointHelper> alignedPoints = new ArrayList<PathPointHelper>();
    // Calculate the angle between the two waypoints
    double dx = end.getX() - start.getX();
    double dy = end.getY() - start.getY();
    double theta = Math.atan2(dy, dx);

    alignedPoints.add(
        new PathPointHelper(
            start.name,
            start.getX(),
            start.getY(),
            Rotation2d.fromRadians(theta),
            start.getHolonomicRotation()));
    alignedPoints.add(
        new PathPointHelper(
            end.name,
            end.getX(),
            end.getY(),
            Rotation2d.fromRadians(theta),
            end.getHolonomicRotation()));
    return alignedPoints;
  }
}
