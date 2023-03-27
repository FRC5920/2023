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

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.Shoot.ShootConfig;
import frc.robot.commands.Shooter.ShooterPresets;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class LinkAndBalanceAutoBuilder {

  /** Initial pose of the robot in the auto routine */
  private static final Pose2d kInitialPose =
      new Pose2d(2.87, 4.9, new Rotation2d(Units.degreesToRadians(-7.0)));

  /** Configuration used to shoot the initial pre-loaded cube at the beginning of the auto */
  private static final ShootConfig kInitialShotConfig = ShooterPresets.CloseShotLow.config;

  /** Max velocity used when following PathPlanner trajectories */
  private static final double kDefaultMaxVelocity = 5.0;

  /** Max acceleration used when following PathPlanner trajectories */
  private static final double kDefaultMaxAcceleration = 8.0;

  /** PathPlanner trajectory file and configuration used to load C1 */
  private static final TrajectoryLoader kAcquireC1Loader =
      new TrajectoryLoader("acquireC1Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
  /** PathPlanner trajectory file and constraints used to shoot C1 */
  private static final TrajectoryLoader kShootC1Loader =
      new TrajectoryLoader("shootC1Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
  /** PathPlanner trajectory file and constraints used to load C2 */
  private static final TrajectoryLoader kAcquireC2Loader =
      new TrajectoryLoader("acquireC2Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
  /** PathPlanner trajectory file and constraints used to shoot C2 */
  private static final TrajectoryLoader kShootC2Loader =
      new TrajectoryLoader("shootC2Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);

  /** PathPlanner trajectory file and constraints used to mount the charging station */
  private static final TrajectoryLoader kMountCSLoader =
      new TrajectoryLoader("mountCSTrajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);

  PathPlannerTrajectory m_acquireC1Trajectory;
  PathPlannerTrajectory m_shootC1Trajectory;
  PathPlannerTrajectory m_acquireC2Trajectory;
  PathPlannerTrajectory m_shootC2Trajectory;
  PathPlannerTrajectory m_mountCSTrajectory;

  /** Initial pose of the bot at the beginning of the auto routine */
  Pose2d m_initialPose;

  /** Trajectories describing motion in the auto routine */
  List<PathPlannerTrajectory> m_trajectories = new ArrayList<>();

  /** Creates an instance of the builder and loads trajectory files */
  public LinkAndBalanceAutoBuilder() {
    // Load PathPlanner trajectory files
    m_acquireC1Trajectory = kAcquireC1Loader.loadTrajectory();
    m_shootC1Trajectory = kShootC1Loader.loadTrajectory();
    m_acquireC2Trajectory = kAcquireC2Loader.loadTrajectory();
    m_shootC2Trajectory = kShootC2Loader.loadTrajectory();
    m_mountCSTrajectory = kMountCSLoader.loadTrajectory();

    m_initialPose = m_acquireC1Trajectory.getInitialPose();

    m_trajectories.add(m_acquireC1Trajectory);
    m_trajectories.add(m_shootC1Trajectory);
    m_trajectories.add(m_acquireC2Trajectory);
    m_trajectories.add(m_shootC2Trajectory);
    m_trajectories.add(m_mountCSTrajectory);
  }

  /**
   * Returns a command that executes the auto routine
   *
   * @param autoType The type of preset auto to build
   * @param botContainer Container used to access robot subsystems
   */
  public CommandBase getCommand(AutoType autoType, RobotContainer botContainer) {
    ShooterPivotSubsystem shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intakeSubsystem = botContainer.intakeSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    CommandBase autoCommands =
        Commands.sequence(new Shoot(kInitialShotConfig, shooterPivotSubsystem, intakeSubsystem));

    return autoCommands;
  }

  public Pose2d getInitialPose() {
    return m_initialPose;
  }

  /** Returns a list containing trajectories used to illustrate motion in the auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
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
            eventMap,
            // true to automatically mirror the path according to alliance color (doesn't work
            // properly)
            false,
            // The drive subsystem
            swerveSubsystem);

    return autoBuilder.fullAuto(trajectory);
  }

  /** A helper class used to store settings and load a Trajectory file */
  private static class TrajectoryLoader {
    /** File name to load from */
    private final String m_trajectoryFilePath;
    /** Max velocity (meters per second) to use when following the path */
    private final double m_maxVelocity;
    /** Max acceleration (meters per second^2) to use when following the path */
    private final double m_maxAcceleration;

    /**
     * Creates a TrajectoryLoader that will load a given pathplanner file and apply given
     * constraints
     *
     * @param trajectoryFileName Name of the trajectory file to load
     * @param maxVelocity Max velocity applied to the loaded trajectory
     * @param maxAcceleration Max acceleration applied to the loaded trajectory
     * @note In the source directory, PathPlanner (.path) files are located under the directory
     *     "src\deploy\pathplanner". These files are automatically copied to a corresponding deploy
     *     directory on the RIO when code is downloaded to the robot. The PathPlanner.loadPath()
     *     function used to load trajectories from these files automatically resolves the
     *     deploy/pathplanner directory on the RIO at runtime and appends a ".path" extension to the
     *     trajectory file name it is passed. The TrajectoryLoader class automatically strips off
     *     any ".path" file extension present in a file name when calling PathPlanner.loadPath().
     */
    public TrajectoryLoader(String trajectoryFileName, double maxVelocity, double maxAcceleration) {
      m_trajectoryFilePath = trajectoryFileName;
      m_maxVelocity = maxVelocity;
      m_maxAcceleration = maxAcceleration;
    }

    /**
     * Loads a trajectory using the object's file name and configuration
     *
     * @throws LoadTrajectoryException on failure to load the trajectory file
     */
    public PathPlannerTrajectory loadTrajectory() {
      final String dotPath = ".path";
      String filename = m_trajectoryFilePath;

      // Strip ".path" file name extension if present because PathPlanner always appends it
      if (filename.endsWith(dotPath)) {
        filename = filename.substring(0, filename.length() - dotPath.length());
      }

      PathPlannerTrajectory trajectory =
          PathPlanner.loadPath(filename, new PathConstraints(m_maxVelocity, m_maxAcceleration));

      if (trajectory == null) {
        throw new RuntimeException(
            String.format("Failed to load trajectory file: `%s`", m_trajectoryFilePath));
      }

      return trajectory;
    }
  }
}
