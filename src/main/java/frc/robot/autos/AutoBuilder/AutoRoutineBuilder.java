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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoBuilder.BalanceStrategy.BalanceMotionConfig;
import frc.robot.autos.AutoBuilder.EscapeStrategy.EscapeMotionConfig;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.ChargingStation;
import frc.robot.autos.AutoConstants.EscapeRoute;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.InitialAction;
import frc.robot.autos.AutoConstants.SecondaryAction;
import frc.robot.commands.Balance;
import frc.robot.commands.Shooter.Shoot.ShootConfig;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.SwerveDrivebase.Swerve.WheelPreset;
import java.util.ArrayList;
import java.util.List;

/** A class used to build auto routines */
public class AutoRoutineBuilder {

  /** Trajectory showing overall paths taken during the auto */
  private List<PathPlannerTrajectory> m_cumulativeTrajectory;

  /** The last command built using build() */
  CommandBase m_builtCommand;

  /** Creates an empty AutoRoutineBuilder object */
  public AutoRoutineBuilder() {
    m_cumulativeTrajectory = new ArrayList<>();
    m_builtCommand = null;
  }

  /**
   * Constructs an auto routine
   *
   * @param botContainer Robot container with subsystems
   */
  public CommandBase build(
      RobotContainer botContainer,
      Grids.ScoringPosition startingPosition,
      InitialAction initialAction,
      EscapeRoute.Route escapeRoute,
      SecondaryAction secondaryAction,
      ChargingStation.BalancePosition balancePosition,
      EscapeMotionConfig escapeMotionConfig,
      BalanceMotionConfig balanceMotionConfig) {
    m_cumulativeTrajectory = new ArrayList<PathPlannerTrajectory>();

    IntakeSubsystem intakeSubsystem = botContainer.intakeSubsystem;
    ShooterPivotSubsystem shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

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

    // Gather commands used to perform the selected initial action
    AutoStrategy initialActionStrategy =
        new InitialActionStrategy(initialAction, startingPosition, botContainer);
    autoCommandGroup.addCommands(initialActionStrategy.getCommand());
    m_cumulativeTrajectory.addAll(initialActionStrategy.getTrajectories());

    // Get commands used to escape the community
    EscapeStrategy escapeStrategy =
        new EscapeStrategy(
            startingPosition, escapeRoute, botContainer.swerveSubsystem, escapeMotionConfig);
    autoCommandGroup.addCommands(escapeStrategy.getCommand());
    m_cumulativeTrajectory.addAll(escapeStrategy.getTrajectories());

    // Get commands used to implement post-escape actions
    switch (secondaryAction) {
      case Balance:
        ShootConfig shootConfig = null; // Set null for no shot while balancing
        BalanceStrategy balanceStrategy =
            new BalanceStrategy(
                balancePosition,
                AllianceFlipUtil.apply(BotOrientation.kFacingField),
                shootConfig,
                () -> escapeStrategy.getFinalPose(),
                botContainer,
                balanceMotionConfig);
        autoCommandGroup.addCommands(balanceStrategy.getCommand());
        m_cumulativeTrajectory.addAll(balanceStrategy.getTrajectories());
        break;
      default:
        break;
    }

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
}
