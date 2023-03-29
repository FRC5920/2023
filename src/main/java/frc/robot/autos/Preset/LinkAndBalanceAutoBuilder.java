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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.utility.BotBoundary.Polygon;
import frc.lib.utility.BotBoundary.PoseLimiter;
import frc.lib.utility.BotBoundary.PoseLimiter.BoundaryPolicy;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.autos.AutoConstants.CargoLocation;
import frc.robot.autos.AutoConstants.FieldCoordinates;
import frc.robot.commands.Balance;
import frc.robot.commands.Shooter.SetShooterAngle;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.Shoot.ShootConfig;
import frc.robot.commands.Shooter.ShooterPresets;
import frc.robot.commands.SimulationPrinter;
import frc.robot.commands.zTarget.AutoIntakeWithZTargeting;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class LinkAndBalanceAutoBuilder {

  /** Initial pose of the robot in the auto routine */
  private static final Pose2d kInitialPose =
      new Pose2d(2.5, 4.8, new Rotation2d(Units.degreesToRadians(-15.77)));

  /** Configuration used to shoot the initial pre-loaded cube at the beginning of the auto */
  private static final ShootConfig kInitialShotConfig = ShooterPresets.CloseShotLow.config;

  /** Default PID gains applied to translation when following trajectories */
  private static final PIDConstants kDefaultTranslationPIDGains = new PIDConstants(8.0, 0.0, 0.2);
  /** Default PID gains applied to rotation when following trajectories */
  private static final PIDConstants kDefaultRotationPIDGains = new PIDConstants(10.0, 0.0, 0.2);

  /** Max velocity used when following PathPlanner trajectories */
  private static final double kDefaultMaxVelocity = 5.0;

  /** Max acceleration used when following PathPlanner trajectories */
  private static final double kDefaultMaxAcceleration = 5.0;

  /** PathPlanner trajectory file and configuration used to load C1 */
  private final TrajectoryLoader m_acquireC1Loader;
  /** PathPlanner trajectory file and constraints used to shoot C1 */
  private final TrajectoryLoader m_shootC1Loader;
  /** PathPlanner trajectory file and constraints used to load C2 */
  private final TrajectoryLoader m_acquireC2Loader;
  /** PathPlanner trajectory file and constraints used to shoot C2 */
  private final TrajectoryLoader m_shootC2Loader;
  /** PathPlanner trajectory file and constraints used to mount the charging station */
  private final TrajectoryLoader m_mountCSLoader;

  /** Creates an instance of the builder and loads trajectory files */
  public LinkAndBalanceAutoBuilder() {
    // Load PathPlanner trajectory files
    m_acquireC1Loader =
        new TrajectoryLoader("acquireC1Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_shootC1Loader =
        new TrajectoryLoader("shootC1Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_acquireC2Loader =
        new TrajectoryLoader("acquireC2Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_shootC2Loader =
        new TrajectoryLoader("shootC2Trajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_mountCSLoader =
        new TrajectoryLoader("mountCSTrajectory", kDefaultMaxVelocity, kDefaultMaxAcceleration);
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

    HashMap<String, Command> intakeEventMap = new HashMap<>();
    intakeEventMap.put(
        "deployPivot", new SetShooterAngle(shooterPivotSubsystem, PivotPresets.Acquire));

    PoseLimiter c1AcquireBoundary = makeAcquireBoundary(CargoLocation.C1, swerveSubsystem::getPose);
    PoseLimiter c2AcquireBoundary = makeAcquireBoundary(CargoLocation.C2, swerveSubsystem::getPose);

    CommandBase autoCommands =
        Commands.sequence(
            // First, a command to reset the robot pose to the initial position
            new SimulationPrinter("Set initial pose"),
            new InstantCommand(
                () -> {
                  swerveSubsystem.zeroGyro();
                  swerveSubsystem.resetOdometry(kInitialPose);
                  botContainer.poseEstimatorSubsystem.setCurrentPose(kInitialPose);
                  // swerveSubsystem.setWheelPreset(WheelPreset.Forward);
                }),
            // Shoot pre-loaded cube
            new SimulationPrinter("<Link+Balance> shoot pre-loaded cargo"),
            new Shoot(kInitialShotConfig, shooterPivotSubsystem, intakeSubsystem),
            // Move to and acquire C1
            new SimulationPrinter("<Link+Balance> move to acquire C1"),
            m_acquireC1Loader.generateTrajectoryCommand(
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            // Use vision and Z-targeting to intake C1
            new SimulationPrinter("<Link+Balance> acquire C1"),
            new AutoIntakeWithZTargeting(
                GameTarget.Cube,
                botContainer.ArmCamera,
                swerveSubsystem,
                shooterPivotSubsystem,
                intakeSubsystem,
                c1AcquireBoundary),
            // Move and shoot C1
            new SimulationPrinter("<Link+Balance> move to shoot C1"),
            m_shootC1Loader.generateTrajectoryCommand(
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new SimulationPrinter("<Link+Balance> shoot C1"),
            new Shoot(ShooterPresets.CloseShotLow, shooterPivotSubsystem, intakeSubsystem),
            // Move to and acquire C2
            new SimulationPrinter("<Link+Balance> move to acquire C2"),
            m_acquireC2Loader.generateTrajectoryCommand(
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            // Use vision and Z-targeting to intake C2
            new SimulationPrinter("<Link+Balance> acquire C2"),
            new AutoIntakeWithZTargeting(
                GameTarget.Cube,
                botContainer.ArmCamera,
                swerveSubsystem,
                shooterPivotSubsystem,
                intakeSubsystem,
                c2AcquireBoundary),
            // Move and shoot C2
            new SimulationPrinter("<Link+Balance> move to shoot C2"),
            m_shootC2Loader.generateTrajectoryCommand(
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new SimulationPrinter("<Link+Balance> shoot C2"),
            new Shoot(ShooterPresets.CloseShotLow, shooterPivotSubsystem, intakeSubsystem),
            // Mount the Charging Station and balance
            new SimulationPrinter("<Link+Balance> mount Charging Station"),
            m_mountCSLoader.generateTrajectoryCommand(
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new SimulationPrinter("<Link+Balance> balance on Charging Station"),
            new Balance(swerveSubsystem));
    return autoCommands;
  }

  public Pose2d getInitialPose() {
    return m_acquireC1Loader.getInitialPose();
  }

  /** Returns a list containing trajectories used to illustrate motion in the auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    Alliance alliance = DriverStation.getAlliance();

    // Return trajectories for display
    List<PathPlannerTrajectory> trajectories = new ArrayList<>();
    trajectories.clear();
    trajectories.add(
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            m_acquireC1Loader.getTrajectory(), alliance));
    trajectories.add(
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            m_shootC1Loader.getTrajectory(), alliance));
    trajectories.add(
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            m_acquireC2Loader.getTrajectory(), alliance));
    trajectories.add(
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            m_shootC2Loader.getTrajectory(), alliance));
    trajectories.add(
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            m_mountCSLoader.getTrajectory(), alliance));

    return trajectories;
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

  public static PoseLimiter makeAcquireBoundary(
      CargoLocation gamepiece, Supplier<Pose2d> poseSupplier) {
    // Get the position of the gamepiece (translated to current alliance)
    Translation2d loc = gamepiece.getLocation();
    Polygon boundary = null;

    switch (DriverStation.getAlliance()) {
      case Blue:
        boundary =
            Polygon.simpleRectangle(
                new Translation2d(FieldCoordinates.xMin, FieldCoordinates.yMax),
                new Translation2d(FieldCoordinates.xCenterline - 0.5, loc.getY() - 0.75));
        break;

      case Red:
        boundary =
            Polygon.simpleRectangle(
                new Translation2d(FieldCoordinates.xCenterline + 0.5, FieldCoordinates.yMax),
                new Translation2d(FieldCoordinates.xMax, loc.getY() - 0.75));
        break;
      case Invalid:
        throw new RuntimeException("Invalid alliance selected");
    }

    return new PoseLimiter(poseSupplier, boundary, BoundaryPolicy.KeepInside);
  }

  /** A helper class used to store settings and load a Trajectory file */
  private static class TrajectoryLoader {
    /** File name to load from */
    private final String m_trajectoryFileName;

    /** Path constraints applied when following the trajectory */
    private final PathConstraints m_pathConstraints;

    /** Trajectory loaded from file */
    private PathPlannerTrajectory m_trajectory;

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
      m_trajectoryFileName = trajectoryFileName;
      m_pathConstraints = new PathConstraints(maxVelocity, maxAcceleration);
      m_trajectory = loadTrajectory(m_trajectoryFileName, m_pathConstraints);
    }

    /** Returns the initial pose of the object's trajectory */
    public Pose2d getInitialPose() {
      return m_trajectory.getInitialHolonomicPose();
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
        HashMap<String, Command> eventMap,
        Swerve swerveSubsystem,
        PIDConstants translationGains,
        PIDConstants rotationGains,
        PathConstraints pathConstraints) {
      return Commands.sequence(
          new SimulationPrinter("<Link+Balance> drive trajectory: " + m_trajectoryFileName),
          buildTrajectoryCommand(
              m_trajectory,
              eventMap,
              swerveSubsystem,
              translationGains,
              rotationGains,
              pathConstraints));
    }
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

    PathPlannerTrajectory trajectory = PathPlanner.loadPath(filename, pathConstraints);

    if (trajectory == null) {
      throw new RuntimeException(
          String.format("Failed to load trajectory file: `%s`", trajectoryFileName));
    }

    return trajectory;
  }
}
