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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.utility.BotLogger.BotLog;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakePreset;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/**
 * IntakeGamepiece uses the Intake subsystem to pull in do this, it a cube gamepiece. To do this, it
 * implements the following sequence: 1.) Ramp up Intake Motors to get the intake rollers spinning
 * 2.) Detect when the rollers have pulled in a gamepiece 3.) Stop intake rollers
 *
 * <p>Detecting when a gamepiece has been pulled into the intake is done by sensing the friction
 * created by the cube when it gets pulled into the rollers. This friction causes the speed of the
 * motors to decrease and the motors to draw more current as they work to run at the commanded
 * speed.
 *
 * <p>Detecting an increase in motor current is complicated by the fact that when the motors are
 * initially started, they draw more current as they work to overcome the inertia of the intake
 * rollers. This causes a spike in electrical current until the rollers reach their commanded speed.
 * Additionally, motor current and speed sensor readings tend to be fairly "noisy". To mitigate
 * these problems, motor current and speed measurements are passed through moving average filters
 * that yield the average measurement over a period of time.
 */
public class IntakeGamepiece extends SequentialCommandGroup {

  /** Amount of time (seconds) to average motor current and speed measurements */
  private static final double kAverageWindowSec = 0.2;

  /**
   * Motor current threshold used to detect when a cube has been loaded into the intake. A cube has
   * been loaded when the average motor current exceeds this threshold.
   */
  private static final double kCurrentThresholdAmps = 65.0;

  /**
   * Motor speed threshold used to detect when a cube has been loaded into the intake. A cube has
   * been loaded when the average motor speed falls below this percentage of the commanded speed.
   */
  private static final double kSpeedThresholdPercent = 60.0;

  /** Number of taps in moving average filters applied to motor measurements */
  private static final int kNumFilterTaps = (int) (kAverageWindowSec / Constants.robotPeriodSec);

  /**
   * Filter used to calculate the average intake motor speed value
   *
   * @remarks This filter must by shared by internal motor ramp-up and gamepiece detection commands
   *     to prevent discontinuities in average motor speed calculation.
   */
  private LinearFilter m_speedAverager = LinearFilter.movingAverage(kNumFilterTaps);

  /** Creates a new IntakeGamepiece. */
  public IntakeGamepiece(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    m_speedAverager.reset();

    addCommands(
        new BotLog.DebugPrintCommand("<IntakeGamepiece> ramp up intake motor"),
        new RampUpIntakeMotors(intakeSubsystem, m_speedAverager, IntakePreset.Acquire.motorSpeed),
        new BotLog.DebugPrintCommand("<IntakeGamepiece> detect gamepiece"),
        new DetectGamepiece(
            intakeSubsystem, m_speedAverager, kSpeedThresholdPercent, kCurrentThresholdAmps),
        new BotLog.DebugPrintCommand("<IntakeGamepiece> stop intake"),
        new InstantCommand(() -> intakeSubsystem.stopIntake()),
        new BotLog.SimDebugPrintCommand("<IntakeGamepiece> reset average filter"),
        new InstantCommand(() -> m_speedAverager.reset()));
  }

  /**
   * Internal command used to ramp up intake motors. Finishes when intake motors are running at a
   * specified target speed.
   */
  private static class RampUpIntakeMotors extends CommandBase {

    /** The Intake subsystem to operate on */
    private final IntakeSubsystem m_intakeSubsystem;

    /** Filter used to calculate the average intake motor speed value */
    private LinearFilter m_averager;

    /** The command will finish once the average motor speed has exceeded this value */
    private final double m_targetMotorSpeedPercent;

    /**
     * Creates an instance of the command
     *
     * @param intakeSubsystem Intake subsytem to operate on
     * @param speedFilter Filter used to calculate average motor speed
     * @param targetSpeed Target motor speed to ramp up to
     */
    RampUpIntakeMotors(
        IntakeSubsystem intakeSubsystem, LinearFilter speedFilter, double targetSpeed) {
      m_intakeSubsystem = intakeSubsystem;
      m_targetMotorSpeedPercent = targetSpeed;
      m_averager = speedFilter;
    }

    @Override
    public void initialize() {
      BotLog.Debugf(
          "<RampUpIntakeMotors> set intake speed to %.0f percent\n", m_targetMotorSpeedPercent);
      m_intakeSubsystem.setSpeedPercent(m_targetMotorSpeedPercent);
    }

    // Called each execution cycle.
    @Override
    public void execute() {}

    /** Returns true when the motor speed exceeds 95% of the target speed */
    @Override
    public boolean isFinished() {
      double averageSpeed = m_averager.calculate(m_intakeSubsystem.getSpeedPercent());
      if (RobotBase.isSimulation()) {
        averageSpeed = m_targetMotorSpeedPercent;
      }

      averageSpeed = m_intakeSubsystem.getSpeedPercent();
      double delta = Math.abs(m_targetMotorSpeedPercent - averageSpeed);

      boolean finished = (delta <= Math.abs(m_targetMotorSpeedPercent * 0.10));
      if (finished) {
        BotLog.Debugf("<RampUpIntakeMotors> finished with speed at %.0f percent\n", averageSpeed);
      }
      return finished;
    }

    @Override
    public void end(boolean interrupted) {}
  }

  /**
   * Internal command used to detect when a gamepiece has been loaded into the intake. Finishes when
   * a gamepiece is detected.
   */
  private static class DetectGamepiece extends CommandBase {

    /** The Intake subsystem to operate on */
    private final IntakeSubsystem m_intakeSubsystem;

    /** If motor speed falls below this threshold, a gamepiece is loaded */
    private final double m_motorSpeedThreshold;

    /** If motor current exceeds this threshold, a gamepiece is loaded */
    private final double m_motorCurrentThreshold;

    /** Filter used to calculate the average intake motor speed */
    private LinearFilter m_speedAverager;

    /** Filter used to calculate the average intake motor current value */
    private LinearFilter m_currentAverager = LinearFilter.movingAverage(kNumFilterTaps);

    /**
     * Creates an instance of the command
     *
     * @param intakeSubsystem Intake subsytem to operate on
     * @param speedFilter Filter used to calculate average motor speed
     * @param currentFilter Filter used to calculate average motor current
     * @param speedThreshold Threshold used to detect a gamepiece using speed
     * @param currentThreshold Threshold used to detect a gamepiece using current
     */
    public DetectGamepiece(
        IntakeSubsystem intakeSubsystem,
        LinearFilter speedFilter,
        double speedThreshold,
        double currentThreshold) {
      m_intakeSubsystem = intakeSubsystem;
      m_motorSpeedThreshold = speedThreshold;
      m_speedAverager = speedFilter;
      m_motorCurrentThreshold = currentThreshold;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      BotLog.Info("<DetectGamepiece> initialized");
      m_currentAverager.reset();
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        BotLog.Info("<DetectGamepiece> interrupted");
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      boolean limitSwitchClosed = m_intakeSubsystem.limitSwitchIsClosed();
      if (limitSwitchClosed) {
        BotLog.Info("<DetectGamepiece> limit switch closed");
      }
      return limitSwitchClosed;
      // || detectUsingSpeed(); // || detectUsingCurrent();
    }

    private boolean detectUsingCurrent() {
      double averageCurrent = m_currentAverager.calculate(m_intakeSubsystem.getMotorCurrentAmps());
      return averageCurrent >= m_motorCurrentThreshold;
    }

    private boolean detectUsingSpeed() {
      double averageSpeed = m_speedAverager.calculate(m_intakeSubsystem.getSpeedPercent());
      return averageSpeed >= m_motorSpeedThreshold;
    }
  }
}
