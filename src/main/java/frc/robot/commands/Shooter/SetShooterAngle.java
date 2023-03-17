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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;

public class SetShooterAngle extends CommandBase {
  /** Tolerance in degrees for the commanded pivot position */
  private static final double kAngleToleranceDeg = 3.0;

  /** Subsystem the command operates on */
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;

  private final double m_pivotDegrees;

  /** Timer used to simulate shot in simulation mode */
  private Timer m_simulationTimer = new Timer();

  /**
   * Creates a new instance of the command that sets the pivot to a specified angle
   *
   * @param pivotDegrees Angle to move the shooter pivot to
   */
  public SetShooterAngle(ShooterPivotSubsystem shooterPivotSubsystem, double pivotDegrees) {
    m_shooterPivotSubsystem = shooterPivotSubsystem;
    m_pivotDegrees = pivotDegrees;
    addRequirements(shooterPivotSubsystem);
  }

  /**
   * Creates a new instance of the command that sets the pivot to a specified angle preset
   *
   * @param pivotDegrees Angle preset to move the shooter pivot to
   */
  public SetShooterAngle(ShooterPivotSubsystem shooterPivotSubsystem, PivotPresets preset) {
    this(shooterPivotSubsystem, preset.angleDegrees);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterPivotSubsystem.setAngleDegrees(m_pivotDegrees);

    if (RobotBase.isSimulation()) {
      System.out.println("Shooter: Pivot to " + String.valueOf(m_pivotDegrees) + " degrees");
      m_simulationTimer.restart();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // In simulation mode, we can't actually shoot anything.  Instead, we approximate
    // the time it takes to shoot.
    if (RobotBase.isSimulation()) {
      boolean finished = m_simulationTimer.hasElapsed(0.5);
      if (finished) {
        System.out.println("Shooter: Pivot to " + String.valueOf(m_pivotDegrees) + " degrees");
      }
      return finished;
    }

    double presentAngleDeg = m_shooterPivotSubsystem.getAngleDegrees();
    double delta = Math.abs(presentAngleDeg - m_pivotDegrees);
    // SmartDashboard.putNumber("PivotTarget", m_pivotDegrees);
    // SmartDashboard.putNumber("PivotAngle", presentAngleDeg);
    // SmartDashboard.putNumber("PivotDelta", delta);
    return delta < kAngleToleranceDeg;
  }
}
