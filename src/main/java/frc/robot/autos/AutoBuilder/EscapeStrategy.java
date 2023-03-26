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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
import java.util.List;

/** The EscapeStrategy class is used to generate auto commands used to escape the community */
public class EscapeStrategy extends AutoStrategy {
  /** Pose where the bot is initially positioned */
  private Grids.ScoringPosition m_startingPosition;

  /** Route the bot will follow to escape from the Community */
  private EscapeRoute.Route m_escapeRoute;

  /** A list of waypoints marking the path of the bot out of the community */
  ArrayList<PathPointHelper> m_waypointList = new ArrayList<PathPointHelper>();

  /** A list of trajectories to follow to escape the community */
  private List<PathPlannerTrajectory> m_escapeTrajectories;

  /** Swerve subsystem used to carry out commands */
  private final Swerve m_swerveSubsystem;

  /** PID gains applied to translation carried out by commands */
  private final PIDGains m_translationPIDGains;
  /** PID gains applied to rotation carried out by commands */
  private final PIDGains m_rotationPIDGains;

  /** Velocity and acceleration constraints observed when moving out of the community */
  private final PathConstraints m_pathConstraints;

  /** The final pose of the robot at the end of the escape */
  private Pose2d m_finalPose;

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
      Swerve swerveSubsystem,
      PIDGains translationPIDGains,
      PIDGains rotationPIDGains,
      double maxVelocity,
      double maxAcceleration) {
    // Initialize common base class
    super(() -> startingPosition.getPose());

    // Set the initial position
    m_startingPosition = startingPosition;
    m_escapeRoute = escapeRoute;
    m_swerveSubsystem = swerveSubsystem;
    m_translationPIDGains = translationPIDGains;
    m_rotationPIDGains = rotationPIDGains;
    m_pathConstraints = new PathConstraints(maxVelocity, maxAcceleration);

    // Pre-generate waypoints used to carry out the auto
    generatePathWaypoints();

    // Generate trajectories used to carry out the auto
    generateEscapeTrajectories();
  }

  /** Returns a list of trajectories used to carry out the auto routine */
  @Override
  public List<PathPlannerTrajectory> getTrajectories() {
    return m_escapeTrajectories;
  }

  /** Returns the final pose after the strategy has executed */
  @Override
  public Pose2d getFinalPose() {
    return m_finalPose;
  }

  /**
   * Generates a command to drive to a series of waypoints to escape the community
   *
   * @param swerveSubsystem Swerve drive subsystem used to drive
   */
  public CommandBase getCommand() {

    SequentialCommandGroup commands = new SequentialCommandGroup();

    for (int idx = 1; idx < m_waypointList.size(); ++idx) {
      PathPointHelper waypoint = m_waypointList.get(idx);
      commands.addCommands(
          new PrintCommand(String.format("Drive to waypoint: %s", waypoint.name)),
          new DriveToWaypoint(
              m_swerveSubsystem,
              new Pose2d(waypoint.getPosition(), waypoint.getHolonomicRotation()),
              DriveToWaypoint.kDefaultPositionToleranceMeters,
              DriveToWaypoint.kDefaultRotationToleranceRadians,
              m_translationPIDGains,
              m_rotationPIDGains,
              DriveToWaypoint.kDefaultPositionConstraints,
              DriveToWaypoint.kDefaultRotationConstraints));
    }

    commands.addCommands(new PrintCommand("My escape is complete"));
    return commands;
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
    Rotation2d dummyTheta = new Rotation2d(); // Placeholder for value filled in later
    PathPointHelper initialWaypoint =
        new PathPointHelper("Initial position", initialPose, dummyTheta);

    // Create a PathPoint for the corner of our escape route.
    final EscapeRoute.Corner corner = EscapeRoute.getCorner(m_escapeRoute);
    final Translation2d cornerPosition = corner.getPosition();
    PathPointHelper cornerPoint =
        new PathPointHelper(
            corner.name(),
            new Pose2d(cornerPosition, initialHolRot),
            dummyTheta); // Heading will get filled in later

    // Generate a waypoint to move into the active lane
    PathPointHelper initialLaneEndpoint =
        new PathPointHelper(
            "initialLaneEndpoint",
            cornerPosition.getX(),
            initialWaypoint.getY(),
            BotOrientation.kFacingField,
            dummyTheta);

    // Create a waypoint for the endpoint of the active escape route
    final EscapeRoute.Endpoint endpoint = EscapeRoute.getEndpoint(m_escapeRoute);
    Pose2d endPose =
        new Pose2d(endpoint.getPosition(), AllianceFlipUtil.apply(BotOrientation.kFacingField));
    PathPointHelper endWaypoint =
        new PathPointHelper(endpoint.name(), endPose, BotOrientation.kFacingField);

    m_waypointList = new ArrayList<>();
    m_waypointList.add(initialWaypoint);
    if (initialLaneEndpoint.distance(initialWaypoint) > BotDimensions.kFootprintWidth) {
      m_waypointList.add(initialLaneEndpoint);
    }
    m_waypointList.add(cornerPoint);
    m_waypointList.add(endWaypoint);

    // Register the final pose
    m_finalPose = new Pose2d(endWaypoint.getPosition(), endWaypoint.getHolonomicRotation());

    // DEBUG: Print the list of path waypoints
    // for (PathPointHelper wp : m_waypointList) {
    //   System.out.println(wp.format());
    // }
  }

  /** Generates a path for the bot to follow to get out of the Grid/community */
  private void generateEscapeTrajectories() {
    // Generate a list of trajectories from the waypoint list
    m_escapeTrajectories = generateTrajectoriesFromWaypoints(m_waypointList, m_pathConstraints);
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
            start.getHolonomicRotation(),
            Rotation2d.fromRadians(theta)));
    alignedPoints.add(
        new PathPointHelper(
            end.name,
            end.getX(),
            end.getY(),
            end.getHolonomicRotation(),
            Rotation2d.fromRadians(theta)));
    return alignedPoints;
  }
}
