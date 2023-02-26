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
import frc.robot.autos.AutoConstants.Lanes;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.AutoConstants.Waypoints;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

import java.util.ArrayList;
import java.util.HashMap;

/** A class used to build auto routines */
public class AutoRoutineBuilder {
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
  private final Grids.ScoringPosition m_startingPosition;
  /** Lane that the bot should use to escape the community/Grids */
  private final Lanes.ID m_escapeLane;
  /** Route the bot will follow to escape from the Community */
  private final EscapeRoute.ID m_escapeRoute;
  /** Waypoint to travel to when escaping the community */
  private final Waypoints.ID m_escapeWaypoint;
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

  AutoRoutineBuilder(
      RobotContainer botContainer,
      Grids.ScoringPosition startingPosition,
      Lanes.ID escapeLane,
      EscapeRoute.ID escapeRoute,
      Waypoints.ID escapeWaypoint) {
    m_startingPosition = startingPosition;
    m_escapeLane = escapeLane;
    m_escapeRoute = escapeRoute;
    m_escapeWaypoint = escapeWaypoint;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

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

  public Command build(RobotContainer botContainer) {
    // Generate an escape trajectory for the given auto parameters
    generateEscapeTrajectory();

    // TODO: concatenate all trajectories together
    m_cumulativeTrajectory = m_escapeTrajectory;

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
    ArrayList<PathPointHelper> escapeWaypoints = new ArrayList<PathPointHelper>();

    Rotation2d initialHolRot = m_startingPosition.pose.getRotation();
    Rotation2d populateLater = new Rotation2d();  // Placeholder for value filled in later
    PathPointHelper initialWaypoint = new PathPointHelper(
      "Initial position", m_startingPosition.pose.getX(), m_startingPosition.pose.getY(), // X, Y
      BotOrientation.kFacingField, // Heading
      initialHolRot); // Holonomic rotation
    escapeWaypoints.add(initialWaypoint);
    
    // ------------------------------------------------------
    // Generate a point that moves the bot into the active
    // lane from the starting position
    PathPointHelper laneWaypoint;
    if (m_escapeLane == Lanes.ID.Outer) {
      // OUTER lane doesn' need to move far from the initial position, but we set the initial
      // heading
      laneWaypoint = new PathPointHelper("Inital Lane Waypoint",
              m_escapeLane.xColumnCenter, initialWaypoint.getY(), // X, Y
              populateLater, // Heading will get filled in later
              initialHolRot); // Holonomic Rotation

    String laneRouteKey = m_escapeLane.name() + m_escapeRoute.name();

    // Determine the transitional waypoint to use
    Waypoints.Transitional transitionalWaypoint = Waypoints.transitionalMap.get(laneRouteKey);
 
    // If corner waypoints are needed, add them
    if (laneWaypoint.getY() != transitionalWaypoint.coordinates.getY()) {
      
    }
    // ------------------------------------------------------
    // Generate a trajectory that moves the bot to the end of the current lane column
    final PathConstraints escapePathConstraints =
        new PathConstraints(kMaxEscapeVelocityMetersPerSec, kMaxEscapeAccelerationMetersPerSec2);
    m_escapeTrajectory =
        PathPlanner.generatePath(
            escapePathConstraints,
            // Set a waypoint at the initial position
            new PathPoint(
                new Translation2d(xInitial, yInitial), // Translation
                BotOrientation.kFacingField, // Heading
                thetaInitial), // Holonomic Rotation
            // Move to a waypoint at the initial point of the active lane
            laneWaypoint,
            // Move to the extreme Y coordinate of the active lane (does nothing if going over CS)
            new PathPoint(
                new Translation2d(
                    m_escapeLane.xColumnCenter,
                    m_escapeLane.getYForRoute(m_escapeRoute)), // Translation
                BotOrientation.kFacingNorth, // Heading
                BotOrientation.kFacingField), // Holonomic rotation

            // Move to a transitional waypoint corresponding to the selected lane
            new PathPoint(
                new Translation2d(
                    transitionalWaypoint.coordinates.getX(),
                    transitionalWaypoint.coordinates.getY()), // Translation
                BotOrientation.kFacingField, // Heading
                BotOrientation.kFacingField) // Holonomic rotation
            );
  }

  public Trajectory getTrajectory() {
    return m_cumulativeTrajectory;
  }


  private static class PathPointHelper extends PathPoint {
    public final String name;

    public PathPointHelper(String nameStr, double x, double y, Rotation2d theta, Rotation2d holRot) {
      super( new Translation2d(x, y), theta, holRot);
      name = nameStr;
    }

    public String format() {
      return String.format("<%s> (x=%.2f, y=%.2f), theta=%.2f deg, holonomic=%.2f deg\n",
        name, position.getX(), position.getY(), heading.getDegrees(), holonomicRotation.getDegrees());
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
