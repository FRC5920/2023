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
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.BotDimensions;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.AutoConstants.Waypoints;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** A class used to build auto routines */
public class AutoRoutineBuilder {
  /** Port to host PathPlanner data on */
  private static final int kPathPlannerServerPort = 22334;

  /** Proportional gain used for translation when following trajectories */
  private static final double kTranslationkP = 5.0;
  /** Integral gain used for translation when following trajectories */
  private static final double kTranslationkI = 5.0;
  /** Derivative gain used for translation when following trajectories */
  private static final double kTranslationkD = 5.0;

  /** Proportional gain used for rotation when following trajectories */
  private static final double kRotationkP = 5.0;
  /** Integral gain used for rotation when following trajectories */
  private static final double kRotationkI = 5.0;
  /** Derivative gain used for rotation when following trajectories */
  private static final double kRotationkD = 5.0;

  /** Maximum velocity of the bot when escaping the community */
  private static final double kMaxEscapeVelocityMetersPerSec = 4.0;
  /** Maximum acceleration of the bot when escaping the community */
  private static final double kMaxEscapeAccelerationMetersPerSec2 = 3.0;

  /** Pose where the bot is initially positioned */
  private Grids.ScoringPosition m_startingPosition;
  /** Route the bot will follow to escape from the Community */
  private EscapeRoute.Route m_escapeRoute;
  /** Waypoint to travel to when escaping the community */
  private Waypoints.ID m_escapeWaypoint;
  /** What to do after escaping the community */
  private SecondaryAction m_secondaryAction;

  /** Trajectory followed when driving away from the substation */
  private PathPlannerTrajectory m_escapeTrajectory;

  /** Trajectory followed to execute a seconary action (e.g. balance, acquire cargo) */
  private PathPlannerTrajectory m_secondaryActionTrajectory;

  /** Trajectory showing the overall path taken by the robot */
  private PathPlannerTrajectory m_cumulativeTrajectory;

  private final HashMap<String, Command> eventMap = new HashMap<>();

  // An AutoBuilder to use for generating swerve trajectory commands */
  SwerveAutoBuilder m_autoBuilder;

  AutoRoutineBuilder(RobotContainer botContainer) {
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    // DEBUG: Start the PathPlanner server for debugging paths
    PathPlannerServer.startServer(kPathPlannerServerPort);

    m_autoBuilder =
        new SwerveAutoBuilder(
            swerveSubsystem::getPose, // Pose2d supplier
            swerveSubsystem
                ::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            swerveSubsystem.getSwerveKinematics(), // SwerveDriveKinematics
            new PIDConstants(
                kTranslationkP,
                kTranslationkI,
                kTranslationkD), // PID constants to correct for translation error (used to create
            // the X and Y PID controllers)
            new PIDConstants(
                kRotationkP,
                kRotationkI,
                kRotationkD), // PID constants to correct for rotation error (used to create the
            // rotation controller)
            swerveSubsystem
                ::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            swerveSubsystem // The drive subsystem. Used to properly set the requirements of path
            // following commands
            );
  }

  public Command build(
      RobotContainer botContainer,
      Grids.ScoringPosition startingPosition,
      EscapeRoute.Route escapeRoute,
      Waypoints.ID escapeWaypoint) {
    m_startingPosition = startingPosition;
    m_escapeRoute = escapeRoute;
    m_escapeWaypoint = escapeWaypoint;

    // Generate an escape trajectory for the given auto parameters
    generateEscapeTrajectory();

    // TODO: concatenate all trajectories together
    m_cumulativeTrajectory = m_escapeTrajectory;

    // Send the cumulative trajectory to the PathPlanner server
    PathPlannerServer.sendActivePath(m_cumulativeTrajectory.getStates());

    SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();

    // First command resets the robot pose to the initial position
    autoCommandGroup.addCommands(
        new InstantCommand(
            () -> {
              botContainer.swerveSubsystem.resetOdometry(m_startingPosition.pose);
              botContainer.poseEstimatorSubsystem.setCurrentPose(m_startingPosition.pose);
            }),
        // Generate a command to escape the community to the selected waypoint
        m_autoBuilder.fullAuto(m_escapeTrajectory));

    return autoCommandGroup;
  }

  /** Generates a path for the bot to follow to get out of the Grid/community */
  private void generateEscapeTrajectory() {
    ArrayList<PathPointHelper> pointList = new ArrayList<PathPointHelper>();

    Rotation2d initialHolRot = m_startingPosition.pose.getRotation();
    Rotation2d populateLater = new Rotation2d(); // Placeholder for value filled in later
    PathPointHelper initialWaypoint =
        new PathPointHelper(
            "Initial position",
            m_startingPosition.pose.getX(),
            m_startingPosition.pose.getY(), // X, Y
            populateLater, // Heading
            initialHolRot); // Holonomic rotation
    pointList.add(initialWaypoint);

    // Generate waypoints to move from the initial position to the corner associated with the
    // selected route.

    // Create a PathPoint for the corner of our escape route.  It won't be added to the list.  But,
    // we may use it to create associated corner waypoints
    // PathPointHelper routeCorner = new PathPointHelper(
    //   "",
    //         EscapeRoute.cornerMap.get(m_escapeRoute).position.getX(),
    //         EscapeRoute.cornerMap.get(m_escapeRoute).position.getY(), // X, Y
    //         populateLater, // Heading will get filled in later
    //         initialHolRot); // Holonomic Rotation
    // );

    // If moving into the inner lane, create corner waypoints to navigate to the waypoint following

    // ------------------------------------------------------
    // Generate a point that moves the bot into the active
    // lane from the starting position
    PathPointHelper laneWaypoint =
        new PathPointHelper(
            "Inital Lane Waypoint",
            EscapeRoute.laneMap.get(m_escapeRoute).xColumnCenter,
            initialWaypoint.getY(), // X, Y
            populateLater, // Heading will get filled in later
            initialHolRot); // Holonomic Rotation
    pointList.add(laneWaypoint);

    // Create a waypoint for the endpoint of the active escape route, but don't add it yet
    EscapeRoute.Endpoint endpoint = EscapeRoute.endpointMap.get(m_escapeRoute);

    PathPointHelper endWaypoint =
        new PathPointHelper(
            "EscapeEndpoint",
            endpoint.coordinates.getX(),
            endpoint.coordinates.getY(),
            BotOrientation.kFacingField,
            initialHolRot);

    // Generate any intermediate corner points needed to reach the target endpoint via a right-angle
    // path
    List<PathPointHelper> cornerPoints = generateCornerPoints(laneWaypoint, endWaypoint);
    for (PathPointHelper cornerPoint : cornerPoints) {
      pointList.add(cornerPoint);
    }

    pointList.add(endWaypoint);

    // Make each waypoint in the list point to the waypoint that follows it
    alignHeadings(pointList);

    // DEBUG: Print the list of path waypoints
    for (PathPointHelper wp : pointList) {
      System.out.println(wp.format());
    }

    // Move helper points into a list of PathPoints because Java ArrayLists are not polymorphic
    ArrayList<PathPoint> escapeList = new ArrayList<PathPoint>();
    for (PathPointHelper point : pointList) {
      escapeList.add(point);
    }

    // Create a trajectory from the waypoints
    PathConstraints escapePathConstraints =
        new PathConstraints(kMaxEscapeVelocityMetersPerSec, kMaxEscapeAccelerationMetersPerSec2);

    m_escapeTrajectory = PathPlanner.generatePath(escapePathConstraints, escapeList);
  }

  /** Returns the overall trajectory of the generated auto routine */
  public Trajectory getTrajectory() {
    return m_cumulativeTrajectory;
  }

  /**
   * Generates a list of waypoints that create a 90-degree corner of a right-angle path between two
   * waypoints.
   *
   * @param start Point of the starting location
   * @param end Point giving the end location
   */
  private static List<PathPointHelper> generateCornerPoints(
      PathPointHelper start, PathPointHelper end) {
    List<PathPointHelper> cornerPoints = new ArrayList<PathPointHelper>();

    // If the distance between start and end is less than half a bot width, there is no need for
    // corners
    if (Math.abs(start.getY() - end.getY()) <= BotDimensions.kHalfFootprintWidth) {
      return cornerPoints;
    }

    // Create the inside of the corner
    double cornerYAdjust =
        (start.getY() < end.getY())
            ? (-1.0 * BotDimensions.kHalfFootprintWidth)
            : BotDimensions.kHalfFootprintWidth;
    cornerPoints.add(
        new PathPointHelper(
            "CornerInside",
            start.getX(),
            (end.getY() + cornerYAdjust),
            new Rotation2d(),
            start.getHolonomicRotation()));

    // Create the outside of the corner
    cornerPoints.add(
        new PathPointHelper(
            "CornerOutside",
            (start.getX() + BotDimensions.kHalfFootprintWidth),
            end.getY(),
            new Rotation2d(),
            start.getHolonomicRotation()));

    // Create the outside of the corner
    return cornerPoints;
  }

  private static void alignHeadings(ArrayList<PathPointHelper> points) {
    for (int idx = 1; idx < points.size(); ++idx) {
      PathPointHelper point = points.get(idx - 1);
      PathPointHelper pointAfter = points.get(idx);

      // Calculate the angle between the two waypoints
      double dx = pointAfter.getX() - point.getX();
      double dy = pointAfter.getY() - point.getY();
      double theta = Math.atan2(dy, dx);

      points.set(
          (idx - 1),
          new PathPointHelper(
              point.name,
              point.getX(),
              point.getY(),
              Rotation2d.fromRadians(theta),
              point.getHolonomicRotation()));
    }
  }

  private static class PathPointHelper extends PathPoint {
    public final String name;

    public PathPointHelper(
        String nameStr, double x, double y, Rotation2d theta, Rotation2d holRot) {
      super(new Translation2d(x, y), theta, holRot);
      name = nameStr;
    }

    public String format() {
      return String.format(
          "<%s> (x=%.2f, y=%.2f), theta=%.2f deg, holonomic=%.2f deg",
          name,
          position.getX(),
          position.getY(),
          heading.getDegrees(),
          holonomicRotation.getDegrees());
    }

    public double getX() {
      return position.getX();
    }

    public double getY() {
      return position.getY();
    }

    public Translation2d getPosition() {
      return position;
    }

    public Rotation2d getHeading() {
      return heading;
    }

    public Rotation2d getHolonomicRotation() {
      return holonomicRotation;
    }
  }
}
