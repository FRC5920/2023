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
package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.IntakePreset;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;

public class Shoot {
  public static final double kShootDurationSec = 0.5;

  public static Command pivotAndShoot(
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem,
      PivotPresets pivotPreset,
      IntakePreset speedPreset) {
    return Commands.sequence(
        new SetShooterAngle(shooterPivotSubsystem, pivotPreset),
        shootAtSpeed(intakeSubsystem, speedPreset, kShootDurationSec));
  }

  public static Command pivotAndShoot(
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem,
      double pivotDegrees,
      double shooterSpeedPercent) {
    return Commands.sequence(
        new SetShooterAngle(shooterPivotSubsystem, pivotDegrees),
        shootAtSpeed(intakeSubsystem, shooterSpeedPercent, 1.5));
  }

  /**
   * Returns a command that runs the shooter at a given preset speed
   *
   * @param intakeSubsystem The intake subsystem used to shoot
   * @param speedPreset Preset speed used for shooting
   * @param durationSec Amount of time in seconds to run the intake when shooting
   */
  public static Command shootAtSpeed(
      IntakeSubsystem intakeSubsystem, IntakePreset speedPreset, double durationSec) {
    return shootAtSpeed(intakeSubsystem, speedPreset.motorSpeed, durationSec);
  }

  /**
   * Returns a command that runs the shooter at a given percentage of full speed
   *
   * @param intakeSubsystem The intake subsystem used to shoot
   * @param speedPreset Speed to run the shooter at as a percentage of full scale
   * @param durationSec Amount of time in seconds to run the intake when shooting
   */
  public static Command shootAtSpeed(
      IntakeSubsystem intakeSubsystem, double speedPercent, double durationSec) {
    return Commands.sequence(
        new RunIntake(intakeSubsystem, speedPercent).raceWith(new WaitCommand(durationSec)));
  }

  private static class RunIntake extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    private final double m_speedPercent;

    /** Creates a new ShootCube. */
    private RunIntake(IntakeSubsystem intakeSubsystem, double speedPercent) {
      m_intakeSubsystem = intakeSubsystem;
      m_speedPercent = speedPercent;
      addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_intakeSubsystem.setSpeedPercent(m_speedPercent);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.stopIntake();
    }
  }
}
