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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.lib.utility.BotLogger.BotLog;
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

  /** Default max velocity used for position constraints */
  public static final double kDefaultMaxVelocity = DriveToWaypoint.kDefaultMaxVelocity;
  /** Default max acceleration used for position constraints */
  public static final double kDefaultMaxAcceleration = DriveToWaypoint.kDefaultMaxAcceleration;
  /** Default control constraints used for the position controller */
  public static final TrapezoidProfile.Constraints kDefaultPositionConstraints =
      new TrapezoidProfile.Constraints(kDefaultMaxVelocity, kDefaultMaxAcceleration);

  /** Default max velocity used for rotation constraints */
  public static final double kDefaultMaxRotVelocity = DriveToWaypoint.kDefaultMaxRotVelocity;
  /** Default max acceleration used for rotation constraints */
  public static final double kDefaultMaxRotAcceleration =
      DriveToWaypoint.kDefaultMaxRotAcceleration;
  /** Default control constraints used for the rotation controller */
  public static final TrapezoidProfile.Constraints kDefaultRotationConstraints =
      new TrapezoidProfile.Constraints(kDefaultMaxRotVelocity, kDefaultMaxRotAcceleration);

  /** Proportional gain used for translation when following trajectories */
  public static final double kDefaultTranslationkP = 8.0;
  /** Integral gain used for translation when following trajectories */
  public static final double kDefaultTranslationkI = 0.0;
  /** Derivative gain used for translation when following trajectories */
  public static final double kDefaultTranslationkD = 0.2;

  /** Default gains used to control translation */
  public static final PIDGains kDefaultTranslationPIDGains =
      new PIDGains(kDefaultTranslationkP, kDefaultTranslationkI, kDefaultTranslationkD);

  /** Proportional gain used for rotation when following trajectories */
  public static final double kDefaultRotationkP = 10.0;
  /** Integral gain used for rotation when following trajectories */
  public static final double kDefaultRotationkI = 0.0;
  /** Derivative gain used for rotation when following trajectories */
  public static final double kDefaultRotationkD = 0.2;

  /** Default gains used to control rotation */
  public static final PIDGains kDefaultRotationPIDGains =
      new PIDGains(kDefaultTranslationkP, kDefaultTranslationkI, kDefaultTranslationkD);

  /** Default motion configuration */
  public static final EscapeMotionConfig kDefaultMotionConfig =
      new EscapeMotionConfig(
          kDefaultTranslationPIDGains,
          kDefaultRotationPIDGains,
          kDefaultMaxVelocity,
          kDefaultMaxAcceleration,
          kDefaultMaxRotVelocity,
          kDefaultMaxRotAcceleration);

  private static final String kAutoName = "<AutoBuilder EscapeStrategy>";

  /** Pose where the bot is initially positioned */
  private Grids.GridPosition m_startingPosition;

  /** Route the bot will follow to escape from the Community */
  private EscapeRoute.Route m_escapeRoute;

  /** A list of waypoints marking the path of the bot out of the community */
  ArrayList<PathPointHelper> m_waypointList = new ArrayList<PathPointHelper>();

  /** A list of trajectories to follow to escape the community */
  private List<PathPlannerTrajectory> m_escapeTrajectories = new ArrayList<>();

  /** Swerve subsystem used to carry out commands */
  private final Swerve m_swerveSubsystem;

  /** PID gains and constraints applied to motion in the Escape strategy */
  EscapeMotionConfig m_motionConfig;

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
      Grids.GridPosition startingPosition,
      EscapeRoute.Route escapeRoute,
      Swerve swerveSubsystem,
      EscapeMotionConfig motionConfig) {
    // Initialize common base class
    super(() -> startingPosition.getPose());

    // Set the initial position
    m_startingPosition = startingPosition;
    m_escapeRoute = escapeRoute;
    m_swerveSubsystem = swerveSubsystem;
    m_motionConfig = motionConfig;

    if (m_escapeRoute != EscapeRoute.Route.StayPut) {
      // Pre-generate waypoints used to carry out the auto
      generatePathWaypoints();

      // Generate trajectories used to carry out the auto
      generateEscapeTrajectories();
    } else {
      m_finalPose = m_startingPosition.getPose();
    }
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
    // Return an empty command for the "Stay Put" option
    if (m_escapeRoute == EscapeRoute.Route.StayPut) {
      return new BotLog.InfoPrintCommand(kAutoName + " Don't escape - stay put");
    }

    SequentialCommandGroup commands = new SequentialCommandGroup();

    for (int idx = 1; idx < m_waypointList.size(); ++idx) {
      PathPointHelper waypoint = m_waypointList.get(idx);
      commands.addCommands(
          new BotLog.InfoPrintCommand(" %s Drive to waypoint: %s", kAutoName, waypoint.name),
          new DriveToWaypoint(
              m_swerveSubsystem,
              new Pose2d(waypoint.getPosition(), waypoint.getHolonomicRotation()),
              DriveToWaypoint.kDefaultPositionToleranceMeters,
              DriveToWaypoint.kDefaultRotationToleranceRadians,
              m_motionConfig.translationPIDGains,
              m_motionConfig.rotationPIDGains,
              m_motionConfig.translationConstraints,
              m_motionConfig.rotationConstraints));
    }

    commands.addCommands(new BotLog.InfoPrintCommand(kAutoName + " My escape is complete"));
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
            BotOrientation.kFacingGrid,
            dummyTheta);

    // Create a waypoint for the endpoint of the active escape route
    final EscapeRoute.Endpoint endpoint = EscapeRoute.getEndpoint(m_escapeRoute);
    Pose2d endPose =
        new Pose2d(endpoint.getPosition(), AllianceFlipUtil.apply(BotOrientation.kFacingGrid));
    PathPointHelper endWaypoint =
        new PathPointHelper(endpoint.name(), endPose, BotOrientation.kFacingGrid);

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
    m_escapeTrajectories = generateTrajectoriesFromWaypoints(m_waypointList);
  }

  /** Creates a list of PathPlannerTrajectories from a list of waypoints */
  private static List<PathPlannerTrajectory> generateTrajectoriesFromWaypoints(
      List<PathPointHelper> waypointList) {

    ArrayList<PathPlannerTrajectory> trajectoryList = new ArrayList<PathPlannerTrajectory>();

    // Move helper points into a list of PathPoints because Java ArrayLists are not polymorphic
    for (int idx = 1; idx < waypointList.size(); ++idx) {
      // Make each waypoint in the list point to the waypoint that follows it
      List<PathPointHelper> pointList =
          alignHeadings(waypointList.get(idx - 1), waypointList.get(idx));
      PathPointHelper start = pointList.get(0);
      PathPointHelper end = pointList.get(1);
      // System.out.printf("Generate trajectory: %s --> %s\n", start.format(), end.format());

      // Generated trajectories are only used for display, so use some placeholder path constraints
      PathConstraints placeHolderPathConstraints = new PathConstraints(1.0, 1.0);
      trajectoryList.add(PathPlanner.generatePath(placeHolderPathConstraints, start, end));
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

  /** Configuration for motion in the Escape strategy */
  public static class EscapeMotionConfig {
    public final PIDGains translationPIDGains;
    public final PIDGains rotationPIDGains;
    public final TrapezoidProfile.Constraints translationConstraints;
    public final TrapezoidProfile.Constraints rotationConstraints;

    public EscapeMotionConfig(
        PIDGains transGains,
        PIDGains rotGains,
        double maxTranslationVelocity,
        double maxTranslationAcceleration,
        double maxRotationVelocity,
        double maxRotationAcceleration) {
      translationPIDGains = transGains;
      rotationPIDGains = rotGains;
      translationConstraints =
          new TrapezoidProfile.Constraints(maxTranslationVelocity, maxTranslationAcceleration);
      rotationConstraints =
          new TrapezoidProfile.Constraints(maxRotationVelocity, maxRotationAcceleration);
    }
  }
}
