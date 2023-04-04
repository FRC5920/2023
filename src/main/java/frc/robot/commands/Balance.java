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
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utility.BotLogger.BotLog;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

public class Balance extends CommandBase {

  private static final double SwerveP = 0.07;
  private static final double SwerveI = 0.00;
  private static final double SwervekD = 0.0;

  private final Rotation2d m_targetRotation;
  private final PIDController xController = new PIDController(SwerveP, SwerveI, SwervekD);
  private final PIDController yController = new PIDController(SwerveP, SwerveI, SwervekD);
  private final PIDController omegaController = new PIDController(SwerveP, SwerveI, SwervekD);

  private final boolean m_balancePerpetually;
  private final Swerve drivetrainSubsystem;

  private final Timer m_simulationTimer;

  /** Creates a command that balances perpetually */
  public Balance(Swerve drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.m_targetRotation = null; // Control to whatever the present yaw is
    xController.setTolerance(3);
    yController.setTolerance(3);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    m_simulationTimer = new Timer();
    m_balancePerpetually = true;
    addRequirements(drivetrainSubsystem);
  }

  /** Creates a command that balances until it has reached a setpoint */
  public Balance(Swerve drivetrainSubsystem, Rotation2d targetRotation) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    m_targetRotation = targetRotation;
    xController.setTolerance(3);
    yController.setTolerance(3);
    omegaController.setTolerance(Units.degreesToRadians(5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    m_simulationTimer = new Timer();
    m_balancePerpetually = false;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // omegaController.reset(drivetrainSubsystem.getYaw().getRadians());
    // xController.reset(drivetrainSubsystem.getRoll());
    // yController.reset(drivetrainSubsystem.getPitch());

    if (RobotBase.isSimulation()) {
      m_simulationTimer.reset();
      m_simulationTimer.start();
    }

    BotLog.Info("<Balance> Balacing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrainSubsystem.getPitch();
    // drivetrainSubsystem.getRoll();
    xController.setSetpoint(0);
    yController.setSetpoint(0);
    // If a target rotation was not given in the constructor, just control to
    // whatever the present yaw is
    omegaController.setSetpoint(
        m_targetRotation != null
            ? m_targetRotation.getRadians()
            : drivetrainSubsystem.getYaw().getRadians());

    // Drive to the target
    var xSpeed = xController.calculate(-drivetrainSubsystem.getPitch().getDegrees());
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(-drivetrainSubsystem.getRoll().getDegrees());
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(drivetrainSubsystem.getYaw().getRadians());
    if (omegaController.atSetpoint()) {
      omegaSpeed = 0;
    }
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
    drivetrainSubsystem.drive(
        new Translation2d(ySpeed, xSpeed).times(Constants.SwerveDrivebaseConstants.maxSpeed * .15),
        omegaSpeed,
        false,
        true);

    // https://www.instructables.com/Self-Balancing-Robot-Using-PID-Algorithm-STM-MC/
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://mcm-frc-docs.readthedocs.io/en/latest/docs/software/commandbased/profilepid-subsystems-commands.html
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BotLog.Infof("<Balance> end: %s", (interrupted ? "interrupted" : "finished"));
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_balancePerpetually) {
      return false;
    }

    boolean finished = false;
    if (RobotBase.isReal()) {
      finished =
          xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
    } else {
      // In simulation mode, simulate Balance with a delay
      finished = m_simulationTimer.hasElapsed(1.0);
    }

    return finished;
  }
}
