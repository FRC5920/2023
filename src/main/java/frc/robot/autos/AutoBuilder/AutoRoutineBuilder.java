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

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.lib.utility.PIDGains;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.InitialAction;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.autos.AutoConstants.Waypoints;
import frc.robot.autos.BumpScore;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.SwerveDrivebase.Swerve.WheelPreset;
import java.util.ArrayList;
import java.util.List;

/** A class used to build auto routines */
public class AutoRoutineBuilder {

  /** Proportional gain used for translation when following trajectories */
  public static final double kDefaultTranslationkP = 10.0;
  /** Integral gain used for translation when following trajectories */
  public static final double kDefaultTranslationkI = 2.0;
  /** Derivative gain used for translation when following trajectories */
  public static final double kDefaultTranslationkD = 5.0;

  /** Proportional gain used for rotation when following trajectories */
  public static final double kDefaultRotationkP = 2.0;
  /** Integral gain used for rotation when following trajectories */
  public static final double kDefaultRotationkI = 1.0;
  /** Derivative gain used for rotation when following trajectories */
  public static final double kDefaultRotationkD = 1.0;

  /** Maximum velocity of the bot when escaping the community */
  private static final double kMaxEscapeVelocityMetersPerSec = 4.0;
  /** Maximum acceleration of the bot when escaping the community */
  private static final double kMaxEscapeAccelerationMetersPerSec2 = 3.0;

  /** What to do after escaping the community */
  private SecondaryAction m_secondaryAction;
  /** Waypoint to travel to when escaping the community */
  private Waypoints.ID m_waypointToMoveTo;

  /** Trajectory followed to execute a seconary action (e.g. balance, acquire cargo) */
  private List<PathPlannerTrajectory> m_secondaryActionTrajectory;

  /** Trajectory showing the overall path taken by the robot */
  private List<PathPlannerTrajectory> m_cumulativeTrajectory;

  /** The last command built using build() */
  Command m_builtCommand = null;

  public AutoRoutineBuilder(Swerve swerveSubsystem) {}

  /**
   * Constructs an auto routine
   *
   * @param botContainer Robot container with subsystems
   */
  public Command build(
      RobotContainer botContainer,
      Grids.ScoringPosition startingPosition,
      InitialAction initialAction,
      EscapeRoute.Route escapeRoute,
      SecondaryAction secondaryAction,
      PIDGains translationPIDGains,
      PIDGains rotationPIDGains,
      boolean doBumpScore) {
    m_cumulativeTrajectory = new ArrayList<PathPlannerTrajectory>();

    SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();
    Pose2d startPosition = startingPosition.getPose();

    autoCommandGroup.addCommands(
        // First, a command to reset the robot pose to the initial position
        new InstantCommand(
            () -> {
              botContainer.swerveSubsystem.resetOdometry(startPosition);
              botContainer.poseEstimatorSubsystem.setCurrentPose(startPosition);
              botContainer.swerveSubsystem.setWheelPreset(WheelPreset.Forward);
            }));

    // Add a command to implement the initial action
    autoCommandGroup.addCommands(
        getInitialActionCommands(botContainer, initialAction, startingPosition));

    EscapeStrategy escapeStrategy =
        new EscapeStrategy(
            startingPosition,
            escapeRoute,
            translationPIDGains,
            rotationPIDGains,
            kMaxEscapeVelocityMetersPerSec,
            kMaxEscapeAccelerationMetersPerSec2);

    // Generate an escape trajectory for the given auto parameters
    // Command escapeCommand = escapeStrategy.buildTrajectoryCommand(botContainer.swerveSubsystem);
    autoCommandGroup.addCommands(
        escapeStrategy.buildWaypointCommandSequence(
            botContainer.swerveSubsystem, translationPIDGains, rotationPIDGains));
    m_cumulativeTrajectory.addAll(escapeStrategy.getTrajectories());

    // Generate command and trajectory for the secondary action
    switch (secondaryAction) {
      case Balance:
        Translation2d endpoint = EscapeRoute.getEndpoint(escapeRoute).getPosition();
        PathPointHelper balanceStart =
            new PathPointHelper(
                "BalanceStart",
                endpoint.getX(),
                endpoint.getY(),
                AllianceFlipUtil.apply(BotOrientation.kFacingField),
                AllianceFlipUtil.apply(BotOrientation.kFacingField));
        BalanceStrategy balanceStrategy = new BalanceStrategy(balanceStart);
        autoCommandGroup.addCommands(
            balanceStrategy.generateCommand(
                botContainer.swerveSubsystem, translationPIDGains, rotationPIDGains));
        m_cumulativeTrajectory.addAll(balanceStrategy.getTrajectories());
        break;
      default:
        break;
    }

    // TODO: concatenate trajectories from all strategies together

    m_builtCommand = autoCommandGroup;
    return m_builtCommand;
  }

  /** Returns the overall trajectory of the generated auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    return m_cumulativeTrajectory;
  }

  /** Returns the last command built using build() */
  public Command getCommand() {
    return m_builtCommand;
  }

  static CommandBase getInitialActionCommands(
      RobotContainer botContainer,
      InitialAction initialAction,
      Grids.ScoringPosition startingPosition) {
    ShooterPivotSubsystem shooterPivot = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intake = botContainer.intakeSubsystem;

    CommandBase actionCommand = null;

    switch (initialAction) {
      case BumpScore:
        actionCommand = new BumpScore(startingPosition, botContainer.swerveSubsystem);
        break;

      case ShootLow:
        actionCommand = Shoot.pivotAndShootLow(shooterPivot, intake);
        break;

      case ShootMid:
        actionCommand = Shoot.pivotAndShootMid(shooterPivot, intake);
        break;

      case ShootHigh:
        actionCommand = Shoot.pivotAndShootHigh(shooterPivot, intake);
        break;
    }

    return actionCommand;
  }
}
