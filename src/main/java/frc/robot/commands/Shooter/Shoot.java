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

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.utility.BotLogger.BotLog;
import frc.robot.subsystems.Intake.IntakePreset;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;

public class Shoot extends SequentialCommandGroup {
  /** Duration in seconds to run the intake while shooting a cube */
  public static final double kShootDurationSec = 0.5;

  /** Class used to communicate parameters used to shoot a cube */
  public static class ShootConfig {
    /** Angle to set the shooter pivot to */
    public double angleDegrees;
    /** Speed (percent of full scale) to run the intake at */
    public double speedPercent;

    /**
     * Creates an instance of a ShootConfig
     *
     * @param _angleDegrees Angle to set pivot to
     * @param _speedPercent Speed to run intake at when shooting in percent of full scale
     */
    public ShootConfig(double _angleDegrees, double _speedPercent) {
      angleDegrees = _angleDegrees;
      speedPercent = _speedPercent;
    }

    /**
     * Creates an instance of a ShootConfig
     *
     * @param anglePreset Preset angle to set pivot to
     * @param _speedPercent Preset speed to run intake at when shooting
     */
    public ShootConfig(PivotPresets anglePreset, IntakePreset speedPreset) {
      this(anglePreset.angleDegrees, speedPreset.motorSpeed);
    }
  }

  /**
   * Creates a Shoot command that uses presets for angle and speed
   *
   * @param pivotPreset Pivot angle preset to use
   * @param speedPreset Shooter speed preset to use
   * @param shooterPivotSubsystem Pivot subsystem to operate on
   * @param intakeSubsystem Intake subsystem to operate on
   */
  public Shoot(
      ShooterPresets preset,
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem) {

    this(
        preset.config,
        String.format("<Shoot> preset=%s", preset.name()),
        shooterPivotSubsystem,
        intakeSubsystem);
  }

  /**
   * Creates a Shoot command that uses a given angle/speed configuration
   *
   * @param config Angle and speed used to shoot
   * @param shooterPivotSubsystem Pivot subsystem to operate on
   * @param intakeSubsystem Intake subsystem to operate on
   */
  public Shoot(
      ShootConfig config,
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem) {

    this(
        config,
        String.format("<Shoot> angle=%s, speed=%s", config.angleDegrees, config.speedPercent),
        shooterPivotSubsystem,
        intakeSubsystem);
  }

  /** Private constructor used to implement the Shoot command sequence */
  private Shoot(
      ShootConfig config,
      String shootMessage,
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem) {

    addCommands(
        new BotLog.DebugPrintCommand(shootMessage),
        new SetShooterAngle(shooterPivotSubsystem, config.angleDegrees),
        new BotLog.DebugPrintCommand(String.format("<Shoot> Take the shot")),
        new RunIntake(intakeSubsystem, config.speedPercent)
            .raceWith(new WaitCommand(kShootDurationSec)),
        new BotLog.DebugPrintCommand(String.format("<Shoot> Shot complete")));
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

      // System.out.println("Shooter: Firing at " + String.valueOf(m_speedPercent));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.stopIntake();
    }
  }
}
