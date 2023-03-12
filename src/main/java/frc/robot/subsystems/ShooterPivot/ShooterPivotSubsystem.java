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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.PIDGains;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;

public class ShooterPivotSubsystem extends SubsystemBase {
  /** Set this constant true to display a dashboard tab for the intake subsystem */
  private static boolean kEnableDashboardTab = true;

  // Motor CAN ID's
  public static final int kPivotMasterMotorCANId = 30;
  public static final int kPivotSlaveMotorCANId = 31;

  /** Gear ratio used to couple pivot motor axles to pivot drive */
  public static final double kPivotGearRatio = 1.0 / 7.0;

  /** Default pivot PID coefficients */
  public static final double kDefaultPivotPID_kFF = 0.0;

  public static final double kDefaultPivotPID_kP = 0.35;
  public static final double kDefaultPivotPID_kI = 0.0;
  public static final double kDefaultPivotPID_kD = 0.05;
  public static final double kDefaultPivotPID_Iz = 0.01;

  public static final PIDGains kDefaultPIDGains =
      new PIDGains(
          kDefaultPivotPID_kP, kDefaultPivotPID_kI, kDefaultPivotPID_kD, kDefaultPivotPID_kFF);

  public static final double kMotionCruiseVelocityDegPerSec = 30.0;
  public static final double kMotionAccelerationDegPerSec2 = 15.0;
  public static final int kMotionSmoothing = 8;

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
  private final WPI_TalonFX m_masterMotor = new WPI_TalonFX(kPivotMasterMotorCANId);

  private final WPI_TalonFX m_slaveMotor = new WPI_TalonFX(kPivotSlaveMotorCANId);
  private final MotorControllerGroup m_pivotMotorGroup =
      new MotorControllerGroup(m_masterMotor, m_slaveMotor);

  /** PID coefficients used for closed-loop control of pivot motor position */
  private PIDGains m_pivotPIDGains = kDefaultPIDGains;

  /** Dashboard tab for the shooter pivot subsystem */
  final ShooterPivotDashboardTab m_dashboardTab;

  public enum PivotPreset {
    /** Angle used to park the intake */
    Park(0, 0.0),
    /** Angle used to acquire a game piece */
    Acquire(1, 193.0),
    /** Angle used when transporting a game piece */
    Transport(2, 70),
    /** Angle used for a short-range shot to the low grid */
    ShortShotLow(3, 170),
    /** Angle used for a short-range shot to the middle grid */
    ShortShotMid(4, 135),
    /** Angle used for a short-range shot to the high grid */
    ShortShotHigh(5, 125),
    /** Angle used for a long-range shot to the low grid */
    LongShotLow(6, 170),
    /** Angle used for a long-range shot to the middle grid */
    LongShotMid(7, 150),
    /** Angle used for a long-range shot to the high grid */
    LongShotHigh(8, 135);

    public final int index;
    public final double angleDegrees;

    private PivotPreset(int idx, double angle) {
      index = idx;
      angleDegrees = angle;
    }
  }

  /** Creates a new ShooterPivot. */
  public ShooterPivotSubsystem() {
    m_dashboardTab = (kEnableDashboardTab) ? new ShooterPivotDashboardTab(this) : null;
    configureMotors();
  }

  /**
   * Moves the pivot to a specified angle in degrees
   *
   * @param degrees Angle in degrees to move the shooter pivot to
   */
  public void setAngleDegrees(double degrees) {
    // Convert RPM to falcon sensor velocity
    double falconTicks = degreesToFalconTicks(degrees);
    SmartDashboard.putNumber("CommandedDegrees", degrees);
    SmartDashboard.putNumber("CommandedTicks", falconTicks);
    m_masterMotor.set(TalonFXControlMode.MotionMagic, falconTicks);
  }

  /**
   * Sets the shooter pivot to a preset angle
   *
   * @param preset Preset angle to set the pivot to
   */
  public void setAnglePreset(PivotPreset preset) {
    setAngleDegrees(preset.angleDegrees);
  }

  /**
   * Returns true if the present pivot position is at a given angle
   *
   * @param angleDeg Position to check for
   * @param toleranceDeg Maximum distance (degrees) in either direction from angleDeg that is
   *     tolerated as being "close enough"
   */
  public boolean isAtAngle(double angleDeg, double toleranceDeg) {
    double targetPosition = m_masterMotor.getClosedLoopTarget(kPIDLoopIdx);
    double presentPosition = m_masterMotor.getSelectedSensorPosition(kPIDLoopIdx);
    return Math.abs(targetPosition - presentPosition) < toleranceDeg;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Resets the encoder count in pivot motors */
  public void zeroPivotPosition() {
    m_masterMotor.setSelectedSensorPosition(0);
    m_slaveMotor.setSelectedSensorPosition(0);
  }

  /**
   * Runs the pivot motors directly at a percentage of maximum output
   *
   * @param speedPercent Speed to run the motor: 0.0 (stop) to 1.0 (forward) or (-1.0) (reverse)
   */
  public void DEBUG_runPivotMotor(double speedPercent) {
    m_masterMotor.set(ControlMode.PercentOutput, speedPercent * kMaxMotorOutputPercent);
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
    m_pivotPIDGains = gains;
    final int timeoutMs = 10;
    applyPIDGains(m_masterMotor, gains);
    applyPIDGains(m_slaveMotor, gains);
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
    // Gather telemetry from front roller motor
    telemetry.masterMotor.sensorPositionTicks = m_masterMotor.getSelectedSensorPosition();
    telemetry.masterMotor.positionDegrees =
        falconTicksToDegrees(telemetry.masterMotor.sensorPositionTicks);
    telemetry.masterMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.masterMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.masterMotor.temperatureCelcius = m_masterMotor.getTemperature();
    telemetry.slaveMotor.sensorPositionTicks = m_masterMotor.getSelectedSensorPosition();
    telemetry.slaveMotor.positionDegrees =
        falconTicksToDegrees(telemetry.slaveMotor.sensorPositionTicks);
    telemetry.slaveMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.slaveMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.slaveMotor.temperatureCelcius = m_masterMotor.getTemperature();
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
    configureMotorControl(m_masterMotor, m_pivotPIDGains, kMaxMotorOutputPercent);
    configureMotorControl(m_slaveMotor, m_pivotPIDGains, kMaxMotorOutputPercent);

    // Zero the internal sensor position on startup
    zeroPivotPosition();
  }

  private static void configureMotorControl(
      WPI_TalonFX motor, PIDGains gains, double maxOutputPercent) {

    // Set up neutral mode behavior
    motor.setNeutralMode(NeutralMode.Brake);
    // Set deadband to super small 0.001 (0.1 %) the default deadband is 0.04 (4 %)
    motor.configNeutralDeadband(0.01, kTimeoutMs);

    /* Config the peak and nominal outputs, 12V means full */
    motor.configNominalOutputForward(0, kTimeoutMs);
    motor.configNominalOutputReverse(0, kTimeoutMs);
    motor.configPeakOutputForward(maxOutputPercent, kTimeoutMs);
    motor.configPeakOutputReverse(-1.0 * maxOutputPercent, kTimeoutMs);

    // Set up closed loop control
    motor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    motor.setSensorPhase(true);
    motor.configAllowableClosedloopError(0, degreesToFalconTicks(2), kTimeoutMs);

    // Select a motion profile slot
    int kSlotIdx = 0;
    motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

    // Set up PID gains
    applyPIDGains(motor, kDefaultPIDGains);
    motor.config_IntegralZone(kPIDLoopIdx, kDefaultPivotPID_Iz);

    // Set acceleration and cruise velocity
    motor.configMotionCruiseVelocity(
        degreesToFalconTicks(kMotionCruiseVelocityDegPerSec), kTimeoutMs);
    motor.configMotionAcceleration(degreesToFalconTicks(kMotionAccelerationDegPerSec2), kTimeoutMs);
    motor.configMotionSCurveStrength(kMotionSmoothing);
  }
}
