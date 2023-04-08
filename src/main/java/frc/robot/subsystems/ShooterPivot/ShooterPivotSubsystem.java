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
package frc.robot.subsystems.ShooterPivot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.BotLogger.BotLog;
import frc.lib.utility.PIDGains;
import frc.robot.commands.Shooter.SetShooterAngle;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;

public class ShooterPivotSubsystem extends SubsystemBase {
  /** Set this constant true to display a dashboard tab for the intake subsystem */
  private static boolean kEnableDashboardTab = false;

  // Motor CAN ID's
  public static final int kPivotMasterMotorCANId = 30;
  public static final int kPivotSlaveMotorCANId = 31;

  /** Gear ratio used to couple pivot motor axles to primary chain sprocket */
  public static final double kPrimaryGearRatio = 1.0 / 21.0;
  /** Gear ratio used to couple secondary chain sprocket to pivot */
  public static final double kSecondaryGearRatio = 12.0 / 15.0;
  /** The combined gear ratio */
  public static final double kPivotGearRatio = kPrimaryGearRatio * kSecondaryGearRatio;

  /** Default pivot PID coefficients */
  public static final double kDefaultPivotPID_kFF = 0.3;

  public static final double kDefaultPivotPID_kP = 0.25;
  public static final double kDefaultPivotPID_kI = 0.0;
  public static final double kDefaultPivotPID_kD = 0.005;
  public static final double kDefaultPivotPID_Iz = 0.01;

  public static final PIDGains kDefaultPIDGains =
      new PIDGains(
          kDefaultPivotPID_kP, kDefaultPivotPID_kI, kDefaultPivotPID_kD, kDefaultPivotPID_kFF);

  public static final double kMotionCruiseVelocityDegPerSec = 200.0;
  public static final double kMotionAccelerationDegPerSec2 = 130.0;
  public static final int kMotionSmoothing = 6;

  /** Peak output (%) that intake motors should run */
  private static final double kMaxMotorOutputPercent = 1.0;

  // FalconFX reports position in sensor ticks
  // 1 revolution = 2048 counts
  private static final double kFalconTicksPerRevolution = 2048.0;

  // Conversion factor for FalconFX sensor ticks to degrees
  private static final double kDegreesPerFalconTick = 360.0 / kFalconTicksPerRevolution;
  private static final double kFalconTicksPerDegree = kFalconTicksPerRevolution / 360.0;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  /** Motors that drive the pivot angle */
  private final WPI_TalonFX m_motors[] =
      new WPI_TalonFX[] {
        new WPI_TalonFX(kPivotMasterMotorCANId), new WPI_TalonFX(kPivotSlaveMotorCANId)
      };

  private final WPI_TalonFX m_masterMotor = m_motors[0];
  private final WPI_TalonFX m_slaveMotor = m_motors[1];

  /** Set to the last requested pivot angle in degrees */
  private double m_requestedPivotAngleDeg = 0.0;

  /** Flag set to indicate that the pivot position should be auto-zeroed */
  private boolean m_pivotZeroIsNeeded = true;

  /** Dashboard tab for the shooter pivot subsystem */
  final ShooterPivotDashboardTab m_dashboardTab;

  /** Creates a new ShooterPivot. */
  public ShooterPivotSubsystem() {
    m_dashboardTab = (kEnableDashboardTab) ? new ShooterPivotDashboardTab(this) : null;
    configureMotors();

    // Zero the internal sensor position on startup
    zeroPivotPositionSensor();
  }

  public CommandBase getDefaultCommand() {
    CommandBase defaultCommand =
        Commands.either(
            new SetShooterAngle(this, PivotPresets.Park),
            new AutoZeroPivot(this).unless(() -> !m_pivotZeroIsNeeded),
            () -> m_requestedPivotAngleDeg > PivotPresets.Park.angleDegrees);

    defaultCommand.addRequirements(this);
    return defaultCommand;
  }

  /**
   * Moves the pivot to a specified angle in degrees
   *
   * @param degrees Angle in degrees to move the shooter pivot to
   */
  public void setAngleDegrees(double degrees) {
    m_masterMotor.set(TalonFXControlMode.MotionMagic, degreesToFalconTicks(degrees));
    m_requestedPivotAngleDeg = degrees;
    // Auto-zero after every pivot command
    m_pivotZeroIsNeeded = true;
  }

  /**
   * Sets the shooter pivot to a preset angle
   *
   * @param preset Preset angle to set the pivot to
   */
  public void setAnglePreset(PivotPresets preset) {
    setAngleDegrees(preset.angleDegrees);
  }

  /** Sets the shooter pivot to its "Parked" position */
  public void park() {
    setAngleDegrees(PivotPresets.Park.angleDegrees);
  }

  /** Returns the measured pivot angle in degrees */
  public double getAngleDegrees() {
    return falconTicksToDegrees(m_masterMotor.getSelectedSensorPosition());
  }

  /** Returns the subsystem's dashboard tab */
  public void registerDashboardTab(DashboardSubsystem dashboardSubsystem) {
    if (kEnableDashboardTab) {
      dashboardSubsystem.add(m_dashboardTab);
    }
  }

  /**
   * Applies given closed-loop gains to motors
   *
   * @param gains Gains to apply
   */
  public void setPIDGains(PIDGains gains) {
    for (WPI_TalonFX motor : m_motors) {
      applyPIDGains(motor, gains);
    }
  }

  /** Telemetry gathered from intake motors */
  public static class MotorTelemetry {
    /** Falcon position sensor value (ticks) */
    double sensorPositionTicks = 0.0;
    /** position in degrees */
    double positionDegrees = 0.0;
    /** Voltage applied to the motor (Volts) */
    double motorVolts = 0.0;
    /** Motor current measurement (Amperes) from the motor */
    double statorCurrentAmps = 0.0;
    /** Motor temperature measurement (Celcius) from the motor */
    double temperatureCelcius = 0.0;
  }

  /** Measurements gathered in the Arm subsystem */
  public static class ShooterPivotTelemetry {
    final MotorTelemetry masterMotor = new MotorTelemetry();
    final MotorTelemetry slaveMotor = new MotorTelemetry();
  }

  /**
   * Populates a telemetry object with measurements from the subsystem
   *
   * @param telemetry Telemetry to populate with measurements from the subsystem
   */
  public void getTelemetry(ShooterPivotTelemetry telemetry) {
    getMotorTelemetry(telemetry.masterMotor, m_masterMotor);
    getMotorTelemetry(telemetry.slaveMotor, m_slaveMotor);
  }

  /**
   * Populates a MotorTelemetry object using values from a given motor
   *
   * @param telemetry MotorTelemetry to populate
   * @param motor Motor to get measurements from
   */
  private static void getMotorTelemetry(MotorTelemetry telemetry, WPI_TalonFX motor) {
    telemetry.sensorPositionTicks = motor.getSelectedSensorPosition();
    telemetry.positionDegrees = falconTicksToDegrees(telemetry.sensorPositionTicks);
    telemetry.motorVolts = motor.getMotorOutputVoltage();
    telemetry.statorCurrentAmps = motor.getStatorCurrent();
    telemetry.temperatureCelcius = motor.getTemperature();
  }

  /**
   * applyPIDGains applies PID gains to a given motor
   *
   * @param motor Motor to apply gains to
   * @param gains Gains to apply
   */
  private static void applyPIDGains(WPI_TalonFX motor, PIDGains gains) {
    motor.config_kF(kPIDLoopIdx, gains.kFF, kTimeoutMs);
    motor.config_kP(kPIDLoopIdx, gains.kP, kTimeoutMs);
    motor.config_kI(kPIDLoopIdx, gains.kI, kTimeoutMs);
    motor.config_kD(kPIDLoopIdx, gains.kD, kTimeoutMs);
  }

  /** Configure motors in the subsystem */
  private void configureMotors() {

    // Reset motors to factory defaults
    m_masterMotor.configFactoryDefault();
    m_slaveMotor.configFactoryDefault();

    // Configure intake motors
    m_slaveMotor.follow(m_masterMotor);
    m_masterMotor.setInverted(false);
    m_slaveMotor.setInverted(true);

    // Configure closed-loop control
    for (WPI_TalonFX motor : m_motors) {
      // Set up neutral mode behavior
      motor.setNeutralMode(NeutralMode.Brake);

      // Set deadband to super small 0.001 (0.1 %) the default deadband is 0.04 (4 %)
      motor.configNeutralDeadband(0.01, kTimeoutMs);

      /* Config the peak and nominal outputs, 12V means full */
      motor.configNominalOutputForward(0, kTimeoutMs);
      motor.configNominalOutputReverse(0, kTimeoutMs);
      motor.configPeakOutputForward(kMaxMotorOutputPercent, kTimeoutMs);
      motor.configPeakOutputReverse(-1.0 * kMaxMotorOutputPercent, kTimeoutMs);

      // Set up closed loop control
      motor.configSelectedFeedbackSensor(
          TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
      motor.setSensorPhase(true);
      // motor.configAllowableClosedloopError(0, degreesToFalconTicks(1), kTimeoutMs);

      // Select a motion profile slot
      int kSlotIdx = 0;
      motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

      // Set up PID gains
      applyPIDGains(motor, kDefaultPIDGains);
      motor.config_IntegralZone(kPIDLoopIdx, kDefaultPivotPID_Iz);

      // Set acceleration and cruise velocity
      motor.configMotionCruiseVelocity(
          degreesToFalconTicks(kMotionCruiseVelocityDegPerSec), kTimeoutMs);
      motor.configMotionAcceleration(
          degreesToFalconTicks(kMotionAccelerationDegPerSec2), kTimeoutMs);
      motor.configMotionSCurveStrength(kMotionSmoothing);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Converts an angle in degrees to Falcon ticks
   *
   * @param degrees Angle in degrees to convert
   * @return Equivalent value in Falcon ticks
   */
  static double degreesToFalconTicks(double degrees) {
    return degrees / kPivotGearRatio * kFalconTicksPerDegree;
  }

  /**
   * Converts a Falcon ticks value to an equivalent angle in degrees
   *
   * @param falconTicks Falcon ticks value to convert
   * @return Equivalent value in degrees
   */
  static double falconTicksToDegrees(double falconTicks) {
    return falconTicks * kPivotGearRatio * kDegreesPerFalconTick;
  }

  /** Returns the raw sensor position in ticks from the pivot motor */
  private double getPositionTicks() {
    return m_masterMotor.getSelectedSensorPosition();
  }

  /** Returns the pivot speed in ticks per 100 ms from the pivot motor */
  private double getPivotMotorVelocityTicks() {
    return m_masterMotor.getSelectedSensorVelocity();
  }

  /** Resets the encoder count in pivot motors */
  private void zeroPivotPositionSensor() {
    for (WPI_TalonFX motor : m_motors) {
      motor.setSelectedSensorPosition(0);
    }
  }

  /** Runs the pivot motor directly at a given speed */
  private void runPivotMotor(double speedPercent) {
    m_masterMotor.set(speedPercent / 100.0);
  }

  private void resetPivotZeroNeeded() {
    m_pivotZeroIsNeeded = false;
  }

  /**
   * Internal command used to auto-zero the pivot sensor position.
   *
   * @remarks Auto-zero assumes that the pivot has been returned to somewhere close to its start
   *     position. It functions by running the pivot motor in open-loop mode in reverse at a low
   *     speed while measuring the average speed of the motor over a short window of time. When the
   *     pivot reaches its park position, it is physically unable to move any further and the
   *     average speed of the pivot motors drops to zero (or very nearly zero). At that point, the
   *     command disengages the pivot motor, zeroes the sensor position, and disengates the pivot
   *     motor.
   */
  private static class AutoZeroPivot extends CommandBase {
    private static final double kAutoZeroSpeedThreshold = 10;

    /** Speed to run the pivot motor at when auto-zeroing */
    private static final double kAutoZeroMotorSpeedPercent = -2.0;

    /** Set to true to enable logging auto-zero start and end */
    private static final boolean kEnableLogging = true;

    /** Pivot subsystem to operate on */
    private final ShooterPivotSubsystem m_shooterPivotSubsystem;

    /** Timer used to time auto-zero operations */
    private Timer m_timer = new Timer();

    /** A filter used to average pivot motor speed over time */
    private final LinearFilter m_speedAverager = LinearFilter.movingAverage(5);

    /** Creates a new autoZeroPivot. */
    public AutoZeroPivot(ShooterPivotSubsystem shooterPivotSubsystem) {
      m_shooterPivotSubsystem = shooterPivotSubsystem;
      addRequirements(shooterPivotSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if (kEnableLogging) {
        BotLog.Infof(
            "<AutoZeroPivot> starting.  Ticks=%.1f", m_shooterPivotSubsystem.getPositionTicks());
      }

      // Run the motor
      m_shooterPivotSubsystem.runPivotMotor(kAutoZeroMotorSpeedPercent);
      m_speedAverager.reset();
      m_timer.restart(); // Start auto-zero timer
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      // Get the speed of the motor in ticks/sec
      double pivotSpeedTicksPerSec = m_shooterPivotSubsystem.getPivotMotorVelocityTicks();
      // Calculate the average speed of the pivot motor
      double averageSpeedTicksPerSec = m_speedAverager.calculate(pivotSpeedTicksPerSec);

      // The command is finished when the motor is running, but its measured speed is zero
      return RobotBase.isSimulation() || (averageSpeedTicksPerSec < kAutoZeroSpeedThreshold);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

      // Disengage the pivot motor when the command is interrupted or ends
      m_shooterPivotSubsystem.runPivotMotor(0.0);

      if (!interrupted) {
        // If auto-zeroing is complete, zero the pivot position sensor
        m_shooterPivotSubsystem.zeroPivotPositionSensor();
        m_shooterPivotSubsystem.resetPivotZeroNeeded();

        if (kEnableLogging) {
          BotLog.Debugf("Pivot auto-zero completed after %.3f seconds", m_timer.get());
        }
      } else {
        if (kEnableLogging) {
          m_speedAverager.reset(); // Reset speed averaging when interrupted
          BotLog.Debug("Pivot auto-zero interrupted");
        }
      }
    }
  }
}
