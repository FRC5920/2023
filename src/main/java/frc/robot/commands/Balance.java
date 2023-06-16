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

/**
 * Balance is a command that adjusts the robot's position on the Charge Station such that it is
 * balanced.
 */
public class Balance extends CommandBase {
  // Gains applied to translation and rotation PID controllers
  private static final double kP = 0.07;
  private static final double kI = 0.00;
  private static final double kD = 0.0;

  /** Tolerance in degrees allowed when controlling robot pitch */
  private static final double kPitchToleranceDeg = 3.0;
  /** Tolerance in degrees allowed when controlling robot roll */
  private static final double kRollToleranceDeg = 3.0;

  /** Maximum speed in meters/sec to move when balancing */
  private static final double kMaxSpeed = Constants.SwerveDrivebaseConstants.maxSpeed * 0.15;

  /** Target angle of rotation to control for or null to control for no angle change */
  private final Rotation2d m_targetRotation;
  /** PID controller used to bring the robot's pitch (rotation about Y-axis) to zero */
  private final PIDController m_pitchController = new PIDController(kP, kI, kD);
  /** PID controller used to bring the robot's roll (rotation about X-axis) to zero */
  private final PIDController m_rollController = new PIDController(kP, kI, kD);
  /** PID controller used to control the robot's yaw/heading */
  private final PIDController m_yawController = new PIDController(kP, kI, kD);

  /** true if the command should not complete; else false to end when a balance has been reached */
  private final boolean m_balancePerpetually;
  /** Swerve drivebase subsystem to operate on */
  private final Swerve m_drivetrainSubsystem;

  /**
   * During simulation mode, balancing is represented by a timer that runs for a preset amount of
   * time
   */
  private final Timer m_simulationTimer = new Timer();

  /**
   * Creates a command that balances perpetually
   *
   * @param driveTrainSubsystem Subsystem used to move the robot
   */
  public Balance(Swerve drivetrainSubsystem) {
    this(drivetrainSubsystem, null, 3.0, true);
  }

  /**
   * Creates a command that balances until it has reached a setpoint
   *
   * @param driveTrainSubsystem Subsystem used to move the robot
   * @param targetYaw Desired robot yaw to maintain for
   */
  public Balance(Swerve drivetrainSubsystem, Rotation2d targetYaw) {
    this(drivetrainSubsystem, targetYaw, 5.0, false);
  }

  /**
   * Constructs an instance of the command
   *
   * @param driveTrainSubsystem Subsystem used to move the robot
   * @param targetYaw Desired robot yaw to maintain for or null to maintain current yaw
   * @param yawToleranceDeg Tolerance in degrees to use when controlling yaw
   * @param perpetual true to balance forever (command doesn't end); else false to end when balanced
   */
  private Balance(
      Swerve drivetrainSubsystem, Rotation2d targetYaw, double yawToleranceDeg, boolean perpetual) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    m_targetRotation = targetYaw;
    m_pitchController.setTolerance(kPitchToleranceDeg);
    m_rollController.setTolerance(kRollToleranceDeg);
    m_yawController.setTolerance(Units.degreesToRadians(yawToleranceDeg));
    m_yawController.enableContinuousInput(-Math.PI, Math.PI);
    m_balancePerpetually = perpetual;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotBase.isSimulation()) {
      m_simulationTimer.reset();
      m_simulationTimer.start();
    }

    BotLog.Info("<Balance> Balacing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pitchController.setSetpoint(0);
    m_rollController.setSetpoint(0);
    // If a target rotation was not given in the constructor, just control to
    // whatever the present yaw is
    m_yawController.setSetpoint(
        m_targetRotation != null
            ? m_targetRotation.getRadians()
            : m_drivetrainSubsystem.getYaw().getRadians());

    double pitchDegrees = m_drivetrainSubsystem.getPitch().getDegrees();
    double rollDegrees = m_drivetrainSubsystem.getRoll().getDegrees();
    double yawRad = m_drivetrainSubsystem.getYaw().getRadians();

    // Drive to the target
    var xSpeed = m_pitchController.calculate(-pitchDegrees);
    if (m_pitchController.atSetpoint()) {
      xSpeed = 0;
    }

    var ySpeed = m_rollController.calculate(-rollDegrees);
    if (m_rollController.atSetpoint()) {
      ySpeed = 0;
    }

    var omegaSpeed = m_yawController.calculate(yawRad);
    if (m_yawController.atSetpoint()) {
      omegaSpeed = 0;
    }

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
    m_drivetrainSubsystem.drive(
        new Translation2d(ySpeed, xSpeed).times(kMaxSpeed), omegaSpeed, false, true);

    // https://www.instructables.com/Self-Balancing-Robot-Using-PID-Algorithm-STM-MC/
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://mcm-frc-docs.readthedocs.io/en/latest/docs/software/commandbased/profilepid-subsystems-commands.html
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BotLog.Infof("<Balance> end: %s", (interrupted ? "interrupted" : "finished"));
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_balancePerpetually) {
      return false;
    }

    // FUTURE:
    //   Detection of a balanced state could be improved if all applicable
    //   controllers must be balanced for some non-trivial amount of time (e.g.
    //   half a second)
    boolean finished = false;
    if (RobotBase.isReal()) {
      finished = m_pitchController.atSetpoint() && m_rollController.atSetpoint();
      if (m_balancePerpetually) {
        finished &= m_yawController.atSetpoint();
      }
    } else {
      // In simulation mode, simulate Balance with a delay
      finished = m_simulationTimer.hasElapsed(1.0);
    }

    return finished;
  }
}
