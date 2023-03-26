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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.lib.utility.PIDGains;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.ChargingStation.BalancePosition;
import frc.robot.autos.AutoConstants.Waypoints;
import frc.robot.commands.Balance;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.Shoot.ShootConfig;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class BalanceStrategy extends AutoStrategy {

  /** Maximum velocity when driving to Charging Station */
  public static final double kMaxVelocity = 6.0;

  /** Maximum velocity when driving to Charging Station */
  public static final double kMaxAcceleration = 8.0;

  // Create a trajectory from the waypoints
  private static final PathConstraints kDefaultPathConstraints =
      new PathConstraints(kMaxVelocity, kMaxAcceleration);

  /** A list of trajectories followed for the auto (for display) */
  private List<PathPlannerTrajectory> m_trajectories;

  /** Balance position on the charging station */
  private final BalancePosition m_balancePosition;

  /** Shooter parameters used to shoot a cube or null if no shot should be taken */
  private final ShootConfig m_shootConfig;

  /** Holonomic rotation at the balance position */
  private final Rotation2d m_balanceHeading;

  /** ShooterPivot subsystem (only used if shooting while balancing) */
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;

  /** Intake subsystem (only used if shooting while balancing) */
  private final IntakeSubsystem m_intakeSubsystem;

  /** Swerve drive subsystem used to carry out the command */
  private final Swerve m_swerveSubsystem;

  /** PID gains applied to translation when moving the bot */
  private final PIDGains m_translationPIDGains;
  /** PID gains applied to rotation when moving the bot */
  private final PIDGains m_rotationPIDGains;

  /**
   * Creates an instance of the strategy
   *
   * @param balancePosition Position on the charging station to drive to and balance
   * @param balanceHeading Holonomic rotation of the bot at the balance position
   * @param shooterParams Shooter parameters if the bot should shoot a cube, or null to not shoot
   * @param initialPoseSupplier Supplier giving the initial pose at the beginning of the strategy
   * @param botContainer Container used to access robot subsystems
   */
  BalanceStrategy(
      BalancePosition balancePosition,
      Rotation2d balanceHeading,
      ShootConfig shooterParams,
      Supplier<Pose2d> initialPoseSupplier,
      RobotContainer botContainer,
      PIDGains translationGains,
      PIDGains rotationGains) {
    super(initialPoseSupplier);
    m_balancePosition = balancePosition;
    m_balanceHeading = balanceHeading;
    m_shootConfig = shooterParams;
    m_shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    m_intakeSubsystem = botContainer.intakeSubsystem;
    m_swerveSubsystem = botContainer.swerveSubsystem;
    m_translationPIDGains = translationGains;
    m_rotationPIDGains = rotationGains;
  }

  @Override
  public Pose2d getFinalPose() {
    return new Pose2d(m_balancePosition.getBalancePosition(), m_balanceHeading);
  }

  /** Returns trajectories for the auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
  }

  /** Generates a command sequence to drive to the Charging Station and balance */
  @Override
  public CommandBase getCommand() {

    // Empty map of events
    HashMap<String, Command> eventMap = new HashMap<>();

    // Set up a PathPlanner auto builder to produce a command for following generated trajectories
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            // Supplier used to get the bot's present pose
            m_swerveSubsystem::getPose,
            // Pose2d consumer, used to reset odometry at the beginning of auto
            (p) -> {},
            m_swerveSubsystem.getSwerveKinematics(), // SwerveDriveKinematics
            // PID gains to correct for translation error
            new PIDConstants(
                m_translationPIDGains.kP, m_translationPIDGains.kI, m_translationPIDGains.kD),
            // PID gains to correct for rotation error
            new PIDConstants(m_rotationPIDGains.kP, m_rotationPIDGains.kI, m_rotationPIDGains.kD),
            // Module states consumer used to output to the drive subsystem
            m_swerveSubsystem::setModuleStates,
            eventMap,
            // true to automatically mirror the path according to alliance color (doesn't work
            // properly)
            false,
            // The drive subsystem
            m_swerveSubsystem);

    // Generate trajectories to move the bot onto the charging station
    generateTrajectoryToChargingStation();

    CommandBase optionalShootCommand =
        (m_shootConfig != null)
            ? new Shoot(m_shootConfig, m_shooterPivotSubsystem, m_intakeSubsystem)
            : new PrintCommand("No shot configured with balance");

    // Return a command sequence used to execute the strategy
    return Commands.sequence(
        // Print command execution
        new PrintCommand(String.format("Drive to charging station and balance")),
        // Drive to the Charging Station
        autoBuilder.fullAuto(m_trajectories),
        // Balance on the Charging Station
        new Balance(m_swerveSubsystem),
        new PrintCommand("bot has balanced"),
        // Add a command to shoot if m_shootConfig is not null
        optionalShootCommand);
  }

  /** Generates trajectories representing the bot's path to the charging station */
  private void generateTrajectoryToChargingStation() {
    ArrayList<PathPlannerTrajectory> trajectoryList = new ArrayList<PathPlannerTrajectory>();
    Translation2d y = Waypoints.ID.Y.getPosition();
    Translation2d cs = m_balancePosition.getBalancePosition();

    // PathPlanner doesn't automatically adjust rotations according to Alliance
    Rotation2d fieldFacing = AllianceFlipUtil.apply(BotOrientation.kFacingField);
    Rotation2d gridFacing = AllianceFlipUtil.apply(BotOrientation.kFacingGrid);

    Pose2d initialPose = m_initialPoseSupplier.get();
    PathPointHelper initialPoint =
        new PathPointHelper(
            "Initial pose",
            initialPose,
            fieldFacing); // Heading needs to be facing field to get the right spline
    PathPointHelper stageAtY =
        new PathPointHelper(
            "Stage at Y",
            y.getX(),
            m_balancePosition.getBalancePosition().getY(),
            gridFacing,
            m_balanceHeading);
    PathPointHelper onCS =
        new PathPointHelper("ChargingStation", cs.getX(), cs.getY(), gridFacing, m_balanceHeading);

    trajectoryList.add(
        PathPlanner.generatePath(kDefaultPathConstraints, initialPoint, stageAtY, onCS));

    m_trajectories = trajectoryList;
  }
}
