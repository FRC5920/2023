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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.autos.AutoConstants.InitialAction;
import frc.robot.autos.BumpScore;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterPresets;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

/** Implements a strategy for an initial action taken when an auto routine first begins */
public class InitialActionStrategy extends AutoStrategy {
  /** The action to take */
  private final InitialAction m_action;
  /** Initial position of the bot when the auto starts */
  private final Grids.ScoringPosition m_initialPosition;
  /** Subsystem used to access the drive base */
  private final Swerve m_swerveSubsystem;
  /** Subsystem used to access the shooter */
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  /** Subsystem used to access the intake */
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates an instance of the strategy
   *
   * @param action The selected initial action to take
   * @param startingPosition Initial position of the bot when the auto begins
   * @param shooterPivotSubsystem The ShooterPivotSubsystem to use
   * @param intakeSubsystem The IntakeSubsystem to use
   */
  public InitialActionStrategy(
      InitialAction action, Grids.ScoringPosition startingPosition, RobotContainer botContainer) {
    super(() -> startingPosition.getPose());
    m_action = action;
    m_initialPosition = startingPosition;
    m_shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    m_intakeSubsystem = botContainer.intakeSubsystem;
    m_swerveSubsystem = botContainer.swerveSubsystem;
  }

  @Override
  public CommandBase getCommand() {
    CommandBase actionCommand = null;

    switch (m_action) {
      case BumpScore:
        actionCommand = new BumpScore(m_initialPosition, m_swerveSubsystem);
        break;

      case ShootLow:
        actionCommand =
            new Shoot(ShooterPresets.RSLSideLow, m_shooterPivotSubsystem, m_intakeSubsystem);
        break;

      case ShootMid:
        actionCommand =
            new Shoot(ShooterPresets.RSLSideMid, m_shooterPivotSubsystem, m_intakeSubsystem);
        break;

      case ShootHigh:
        actionCommand =
            new Shoot(ShooterPresets.RSLSideHigh, m_shooterPivotSubsystem, m_intakeSubsystem);
        break;
    }

    return actionCommand;
  }

  @Override
  public Pose2d getFinalPose() {
    return m_initialPosition.getPose();
  }
}
