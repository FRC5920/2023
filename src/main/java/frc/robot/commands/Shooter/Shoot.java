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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SimulationPrinter;
import frc.robot.subsystems.Intake.IntakePreset;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;

public class Shoot extends SequentialCommandGroup {
  public static final double kShootDurationSec = 0.5;

  /**
   * Creates a Shoot command that uses presets for angle and speed
   *
   * @param pivotPreset Pivot angle preset to use
   * @param speedPreset Shooter speed preset to use
   * @param shooterPivotSubsystem Pivot subsystem to operate on
   * @param intakeSubsystem Intake subsystem to operate on
   */
  public Shoot(
      PivotPresets pivotPreset,
      IntakePreset speedPreset,
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem) {

    this(
        new ShootConfig(pivotPreset.angleDegrees, speedPreset.motorSpeed),
        String.format("<Shoot> angle=%s, speed=%s", pivotPreset.name(), speedPreset.name()),
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

  /**
   * Private constructor used to implement the Shoot command sequence
   *
   * @param config Angle and speed used to shoot
   * @param shootMessage Message to print prior to shooting
   * @param shooterPivotSubsystem Pivot subsystem to operate on
   * @param intakeSubsystem Intake subsystem to operate on
   */
  private Shoot(
      ShootConfig config,
      String shootMessage,
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem) {

    addCommands(
        new SimulationPrinter(shootMessage),
        new SetShooterAngle(shooterPivotSubsystem, config.angleDegrees),
        new SimulationPrinter(String.format("<Shoot> Take the shot")),
        shootAtSpeed(intakeSubsystem, config.speedPercent, kShootDurationSec),
        new SimulationPrinter(String.format("<Shoot> Shot complete")));
  }

  /** Class used to bundle together parameters used to shoot a cube */
  public static class ShootConfig {
    /** Angle to set the shooter pivot to */
    public double angleDegrees;
    /** Speed (percent of full scale) to run the intake at */
    public double speedPercent;

    /** Creates an instance of a ShootConfig */
    public ShootConfig(double _angleDegrees, double _speedPercent) {
      angleDegrees = _angleDegrees;
      speedPercent = _speedPercent;
    }
  }

  public static CommandBase pivotAndShoot(
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem,
      PivotPresets pivotPreset,
      IntakePreset speedPreset) {
    return new SimulationPrinter(
            String.format(
                "<pivotAndShoot> pivot=%s, speed=%s", pivotPreset.name(), speedPreset.name()))
        .andThen(new SetShooterAngle(shooterPivotSubsystem, pivotPreset))
        .andThen(new SimulationPrinter(String.format("<pivotAndShoot> Take the shot")))
        .andThen(shootAtSpeed(intakeSubsystem, speedPreset, kShootDurationSec))
        .andThen(new SimulationPrinter(String.format("<pivotAndShoot> Shot complete")));
  }

  public static CommandBase pivotAndShoot(
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem,
      double pivotDegrees,
      double shooterSpeedPercent) {
    CommandBase command =
        Commands.sequence(
            new SetShooterAngle(shooterPivotSubsystem, pivotDegrees),
            shootAtSpeed(intakeSubsystem, shooterSpeedPercent, 1.5));
    command.addRequirements(shooterPivotSubsystem, intakeSubsystem);
    return command;
  }

  public static CommandBase pivotAndShootLow(
      ShooterPivotSubsystem shooterPivotSubsystem, IntakeSubsystem intakeSubsystem) {
    return pivotAndShoot(
        shooterPivotSubsystem,
        intakeSubsystem,
        PivotPresets.CloseShotLow,
        IntakePreset.CloseShotLow);
  }

  public static CommandBase pivotAndShootMid(
      ShooterPivotSubsystem shooterPivotSubsystem, IntakeSubsystem intakeSubsystem) {
    return pivotAndShoot(
        shooterPivotSubsystem,
        intakeSubsystem,
        PivotPresets.CloseShotMid,
        IntakePreset.CloseShotMid);
  }

  public static CommandBase pivotAndShootHigh(
      ShooterPivotSubsystem shooterPivotSubsystem, IntakeSubsystem intakeSubsystem) {
    return pivotAndShoot(
        shooterPivotSubsystem,
        intakeSubsystem,
        PivotPresets.CloseShotHigh,
        IntakePreset.CloseShotHigh);
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

      // System.out.println("Shooter: Firing at " + String.valueOf(m_speedPercent));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.stopIntake();
    }
  }
}
