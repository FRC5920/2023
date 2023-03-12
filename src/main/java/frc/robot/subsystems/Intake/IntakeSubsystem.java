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
package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.utility.PIDGains;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;

/** Subsystem for managing the cargo intake at the end of the robot arm */
public class IntakeSubsystem extends SubsystemBase {
  /** Set this constant true to display a dashboard tab for the intake subsystem */
  private static boolean kEnableDashboardTab = true;

  // Motor CAN ID's
  public static final int kIntakeMasterMotorCANId = 40;
  public static final int kIntakeSlaveMotorCANId = 41;

  /** Gear ratio used to couple intake motor axles to rollers */
  public static final double kIntakeGearRatio = 1.0 / 5.0;

  /** Default speed PID coefficients */
  public static final double kDefaultPID_kFF = 0.22;

  public static final double kDefaultPID_kP = 0.22;
  public static final double kDefaultPID_kI = 0.002;
  public static final double kDefaultPID_kD = 0;
  public static final double kDefaultPID_Iz = 100;

  public static final PIDGains kDefaultPIDGains =
      new PIDGains(kDefaultPID_kP, kDefaultPID_kI, kDefaultPID_kD, kDefaultPID_kFF);

  /** Peak output (%) that intake motors should run */
  private static final double kMaxMotorOutputPercent = 0.2;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  /** Motors that drive the intake rollers */
  private final WPI_TalonFX m_masterMotor = new WPI_TalonFX(kIntakeMasterMotorCANId);

  private final WPI_TalonFX m_slaveMotor = new WPI_TalonFX(kIntakeSlaveMotorCANId);
  private final MotorControllerGroup m_intakeMotorGroup =
      new MotorControllerGroup(m_masterMotor, m_slaveMotor);

  /** PID coefficients used for closed-loop control of intake motor speed */
  private PIDGains m_speedPIDGains = kDefaultPIDGains;

  /** Dashboard tab for the intake subsystem */
  final IntakeDashboardTab m_dashboardTab;

  public enum IntakePreset {
    CubeIntake(0, -700.0),
    ConeIntake(1, -800.0),
    CloseShotLow(2, 700),
    CloseShotMid(3, 1000),
    CloseShotHigh(4, 1300);

    public final int index;
    public final double motorRPM;

    private IntakePreset(int idx, double rpm) {
      index = idx;
      motorRPM = rpm;
    }
  }

  /** Creates an instance of the subsystem */
  public IntakeSubsystem() {
    m_dashboardTab = (kEnableDashboardTab) ? new IntakeDashboardTab(this) : null;
    configureMotors();
  }

  /**
   * Activates intake motors at a specified speed preset
   *
   * @param preset Preset to set the intake motors to
   */
  public void activatePreset(IntakePreset preset) {
    setRPM(preset.motorRPM);
  }

  /**
   * Sets the intake motor speed to a given RPM
   *
   * @param rpm Speed (RPM) to run the intake at
   * @remarks Negative RPM values pull a game piece in; positive push it out.
   */
  public void setRPM(double rpm) {
    // Convert RPM to falcon sensor velocity
    double falconVelocity = Conversions.RPMToFalcon(rpm, kIntakeGearRatio);
    m_masterMotor.set(ControlMode.Velocity, falconVelocity);
  }

  /** Deactivates intake motors */
  public void stopIntake() {
    setRPM(0.0);
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
    m_speedPIDGains = gains;
    applyPIDGains(m_masterMotor, gains);
    applyPIDGains(m_slaveMotor, gains);
  }

  /** Telemetry gathered from intake motors */
  public static class MotorTelemetry {
    /** Velocity of the motor in revolutions per minute (RPM) */
    double motorRPM = 0.0;

    /** Voltage applied to the motor (Volts) */
    double motorVolts = 0.0;

    /** Motor current measurement (Amperes) from the motor */
    double statorCurrentAmps = 0.0;

    /** Motor temperature measurement (Celcius) from the motor */
    double temperatureCelcius = 0.0;
  }

  /** Measurements gathered in the Arm subsystem */
  public static class IntakeSubsystemTelemetry {
    final MotorTelemetry masterMotor = new MotorTelemetry();
    final MotorTelemetry slaveMotor = new MotorTelemetry();
  }

  /**
   * Populates a telemetry object with measurements from the subsystem
   *
   * @param telemetry Telemetry to populate with measurements from the subsystem
   */
  public void getTelemetry(IntakeSubsystemTelemetry telemetry) {
    telemetry.masterMotor.motorRPM =
        Conversions.falconToRPM(
            m_masterMotor.getSelectedSensorVelocity(kPIDLoopIdx), kIntakeGearRatio);
    telemetry.masterMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.masterMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.masterMotor.temperatureCelcius = m_masterMotor.getTemperature();
    telemetry.slaveMotor.motorRPM =
        Conversions.falconToRPM(
            m_slaveMotor.getSelectedSensorVelocity(kPIDLoopIdx), kIntakeGearRatio);
    telemetry.slaveMotor.motorVolts = m_slaveMotor.getMotorOutputVoltage();
    telemetry.slaveMotor.statorCurrentAmps = m_slaveMotor.getStatorCurrent();
    telemetry.slaveMotor.temperatureCelcius = m_slaveMotor.getTemperature();
  }

  /**
   * Applies PID gains to a motor
   *
   * @param motor Motor to apply gains to
   * @param gains PID gains to apply
   */
  private static void applyPIDGains(WPI_TalonFX motor, PIDGains gains) {
    motor.config_kF(kPIDLoopIdx, gains.kFF, kTimeoutMs);
    motor.config_kP(kPIDLoopIdx, gains.kP, kTimeoutMs);
    motor.config_kI(kPIDLoopIdx, gains.kI, kTimeoutMs);
    motor.config_kD(kPIDLoopIdx, gains.kD, kTimeoutMs);
  }

  /**
   * Runs the intake motors directly at a percentage of maximum output
   *
   * @param speedPercent Speed to run the motor: 0.0 (stop) to 1.0 (forward) or (-1.0) (reverse)
   */
  public void DEBUG_setMotorPercent(double speedPercent) {
    m_intakeMotorGroup.set(speedPercent * kMaxMotorOutputPercent);
  }

  /** Configure motors in the subsystem */
  private void configureMotors() {

    // Reset motors to factory defaults
    m_masterMotor.configFactoryDefault();
    m_slaveMotor.configFactoryDefault();

    // Configure master to follow slave
    m_slaveMotor.follow(m_masterMotor);

    // Configure motor direction such that positive values push out and negative pull in
    m_masterMotor.setInverted(true);
    m_slaveMotor.setInverted(TalonFXInvertType.OpposeMaster);

    // Configure motor control
    configureMotorControl(m_masterMotor, m_speedPIDGains, kMaxMotorOutputPercent);
    configureMotorControl(m_slaveMotor, m_speedPIDGains, kMaxMotorOutputPercent);
  }

  private static void configureMotorControl(
      WPI_TalonFX motor, PIDGains gains, double maxOutputPercent) {
    // Configure neutral deadband to be tighter than the default
    motor.configNeutralDeadband(0.001);

    // Configure the motor to use its internal velocity sensor
    motor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    // Configure peak and nominal outputs
    motor.configNominalOutputForward(0, kTimeoutMs);
    motor.configNominalOutputReverse(0, kTimeoutMs);
    motor.configPeakOutputForward(maxOutputPercent, kTimeoutMs);
    motor.configPeakOutputReverse(-1.0 * maxOutputPercent, kTimeoutMs);

    applyPIDGains(motor, gains);
    motor.config_IntegralZone(kPIDLoopIdx, kDefaultPID_Iz);
  }

  /** Called by the scheduler to service the subsystem */
  @Override
  public void periodic() {}
}
