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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  public static final double kPivotGearRatio = 1.0 / 1.0;

  /** Default pivot PID coefficients */
  public static final double kDefaultPivotPID_kFF = 0.22;

  public static final double kDefaultPivotPID_kP = 0.22;
  public static final double kDefaultPivotPID_kI = 0.002;
  public static final double kDefaultPivotPID_kD = 0;
  public static final double kDefaultPivotPID_Iz = 100;

  public static final PIDGains kDefaultPIDGains =
      new PIDGains(
          kDefaultPivotPID_kP, kDefaultPivotPID_kI, kDefaultPivotPID_kD, kDefaultPivotPID_kFF);

  /** Peak output (%) that intake motors should run */
  private static final double kMaxMotorOutputPercent = 0.2;

  // FalconFX reports position in sensor ticks
  // 1 revolution = 2048 counts
  private final double kFalconTicksPerRevolution = 2048.0;

  // Conversion factor for FalconFX sensor ticks to degrees
  private final double kFalconTicksPerDegree = kFalconTicksPerRevolution / 180.0;

  private final int kPIDLoopIdx = 0;

  /** Motors that drive the pivot angle */
  private final WPI_TalonFX m_masterMotor = new WPI_TalonFX(kPivotMasterMotorCANId);

  private final WPI_TalonFX m_slaveMotor = new WPI_TalonFX(kPivotSlaveMotorCANId);
  private final MotorControllerGroup m_pivotMotorGroup =
      new MotorControllerGroup(m_masterMotor, m_slaveMotor);

  /** PID coefficients used for closed-loop control of pivot motor position */
  private PIDGains m_pivotPIDGains = kDefaultPIDGains;

  /** Dashboard tab for the shooter pivot subsystem */
  final ShooterPivotDashboardTab m_dashboardTab = new ShooterPivotDashboardTab(this);

  public enum PivotPreset {
    /** Angle used to acquire a game piece */
    Acquire(0, 0),
    /** Angle used when transporting a game piece */
    Transport(1, 45),
    /** Angle used for a short-range shot to the low grid */
    ShortShotLow(2, 0),
    /** Angle used for a short-range shot to the middle grid */
    ShortShotMid(3, 50),
    /** Angle used for a short-range shot to the high grid */
    ShortShotHigh(4, 60),
    /** Angle used for a long-range shot to the low grid */
    LongShotLow(5, 20),
    /** Angle used for a long-range shot to the middle grid */
    LongShotMid(6, 30),
    /** Angle used for a long-range shot to the high grid */
    LongShotHigh(7, 45);

    public final int index;
    public final double angleDegrees;

    private PivotPreset(int idx, double angle) {
      index = idx;
      angleDegrees = angle;
    }
  }

  /** Creates a new ShooterPivot. */
  public ShooterPivotSubsystem() {
    configureMotors();
  }

  /**
   * Sets the shooter pivot to a preset angle
   *
   * @param preset Preset angle to set the pivot to
   */
  private void runIntake(PivotPreset preset) {
    setAngleDegrees(preset.angleDegrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Moves the pivot to a specified angle in degrees
   *
   * @param degrees Angle in degrees to move the shooter pivot to
   */
  public void setAngleDegrees(double degrees) {
    // Convert RPM to falcon sensor velocity
    double falconTicks = degrees * kFalconTicksPerDegree * kPivotGearRatio;
    m_masterMotor.set(ControlMode.Position, falconTicks);
  }

  /**
   * Runs the pivot motors directly at a percentage of maximum output
   *
   * @param speedPercent Speed to run the motor: 0.0 (stop) to 1.0 (forward) or (-1.0) (reverse)
   */
  public void DEBUG_runPivotMotor(double speedPercent) {
    m_pivotMotorGroup.set(speedPercent);
  }

  /** Returns the subsystem's dashboard tab */
  public void registerDashboardTab(DashboardSubsystem dashboardSubsystem) {
    if (kEnableDashboardTab) {
      dashboardSubsystem.add(m_dashboardTab);
    }
  }

  /** Configure motors in the subsystem */
  private void configureMotors() {
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    // Configure intake motors
    m_slaveMotor.follow(m_masterMotor);
    m_masterMotor.setNeutralMode(NeutralMode.Brake);
    applyPIDGains(m_masterMotor, kDefaultPIDGains);
    m_masterMotor.config_IntegralZone(kPIDLoopIdx, kDefaultPivotPID_Iz);
    m_masterMotor.configPeakOutputForward(kMaxMotorOutputPercent, kTimeoutMs);
    m_masterMotor.configPeakOutputReverse(-1.0 * kMaxMotorOutputPercent, kTimeoutMs);

    // Configure pivot motors
    m_slaveMotor.follow(m_masterMotor);
    m_slaveMotor.setNeutralMode(NeutralMode.Brake);
    applyPIDGains(m_slaveMotor, kDefaultPIDGains);
    m_slaveMotor.config_IntegralZone(kPIDLoopIdx, kDefaultPivotPID_Iz);
    m_slaveMotor.configPeakOutputForward(kMaxMotorOutputPercent, kTimeoutMs);
    m_slaveMotor.configPeakOutputReverse(-1.0 * kMaxMotorOutputPercent, kTimeoutMs);
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
    telemetry.masterMotor.sensorPositionTicks =
        telemetry.masterMotor.sensorPositionTicks / kFalconTicksPerDegree;
    telemetry.masterMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.masterMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.masterMotor.temperatureCelcius = m_masterMotor.getTemperature();
    telemetry.slaveMotor.sensorPositionTicks = m_masterMotor.getSelectedSensorPosition();
    telemetry.slaveMotor.sensorPositionTicks =
        telemetry.slaveMotor.sensorPositionTicks / kFalconTicksPerDegree;
    telemetry.slaveMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.slaveMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.slaveMotor.temperatureCelcius = m_masterMotor.getTemperature();
  }

  private static void applyPIDGains(WPI_TalonFX motor, PIDGains gains) {
    final int timeoutMs = 10;
    final int kPIDLoopIdx = 0;
    motor.config_kF(kPIDLoopIdx, gains.kFF, timeoutMs);
    motor.config_kP(kPIDLoopIdx, gains.kP, timeoutMs);
    motor.config_kI(kPIDLoopIdx, gains.kI, timeoutMs);
    motor.config_kD(kPIDLoopIdx, gains.kD, timeoutMs);
  }
}
