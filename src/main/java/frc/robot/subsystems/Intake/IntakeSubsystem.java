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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.PIDGains;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;

/** Subsystem for managing the cargo intake at the end of the robot arm */
public class IntakeSubsystem extends SubsystemBase {
  /** Set this constant true to display a dashboard tab for the intake subsystem */
  private static boolean kEnableDashboardTab = true;

  public static final int kMasterMotorCANId = 6;
  public static final int kSlaveMotorCANId = 6;

  public static final double kIntakeGearRatio = 1.0 / 5.0;

  /** Default PID coefficients */
  public static final double kDefaultkFF = 0.22;

  public static final double kDefaultkP = 0.22;
  public static final double kDefaultkI = 0.002;
  public static final double kDefaultkD = 0;
  public static final double kDefaultkIz = 100;

  public static final PIDGains kDefaultPIDGains = new PIDGains(kDefaultkP, kDefaultkI, kDefaultkD);

  /** Default speeds used for motor presets */
  public static final double kDefaultCubeIntakeRPM = 0.1;

  public static final double kDefaultConeIntakeRPM = 0.2;
  public static final double kDefaultPlaceLowRPM = -0.1;
  public static final double kDefaultPlaceMidRPM = -0.25;
  public static final double kDefaultPlaceHighRPM = -0.35;

  /** Peak output (%) that intake motors should run */
  private static final double kMaxOutputPercent = 0.2;

  private final int kPIDLoopIdx = 0;

  // FalconFX reports velocity in counts per 100ms
  // 1 revolution = 2048 counts
  // 1 minutes = 60 * 10 * 100ms
  // conversion is  600  / 2048
  private final double kFalconTicksToRPM = 600.0 / 2048.0;

  /** Motors that drive the intake rollers */
  private final WPI_TalonFX m_masterMotor = new WPI_TalonFX(kMasterMotorCANId);

  private final WPI_TalonFX m_slaveMotor = new WPI_TalonFX(kSlaveMotorCANId);
  private final MotorControllerGroup m_intakeMotorGroup =
      new MotorControllerGroup(m_masterMotor, m_slaveMotor);

  /** PID coefficients used for closed-loop control of motor speed */
  private PIDGains m_PIDGains = kDefaultPIDGains;

  /** Dashboard tab for the intake subsystem */
  final IntakeDashboardTab m_dashboardTab = new IntakeDashboardTab(this);

  /** Motor speed used for intake of cubes */
  private double[] m_intakePresetRPMs =
      new double[] {
        kDefaultCubeIntakeRPM,
        kDefaultConeIntakeRPM,
        kDefaultPlaceLowRPM,
        kDefaultPlaceMidRPM,
        kDefaultPlaceHighRPM
      };

  public enum IntakePreset {
    CubeIntake(0),
    ConeIntake(1),
    PlaceLow(2),
    PlaceMid(3),
    PlaceHigh(4);

    public final int index;

    private IntakePreset(int idx) {
      index = idx;
    }
  }

  /** Creates an instance of the subsystem */
  public IntakeSubsystem() {
    configureMotors();
  }

  /**
   * Activates intake motors at a specified speed preset
   *
   * @param preset Preset to set the intake motors to
   */
  private void activate(IntakePreset preset) {
    setIntakeRPM(m_intakePresetRPMs[preset.index]);
  }

  /** Deactivates intake motors */
  public void deactivate() {
    m_intakeMotorGroup.set(0.0);
  }

  /** Called by the scheduler to service the subsystem */
  @Override
  public void periodic() {}

  /**
   * Sets the intake motor speed in RPM
   *
   * @param rpm Speed to run the intake at in RPM
   */
  public void setIntakeRPM(double rpm) {
    // Convert RPM to falcon sensor velocity
    double falconVelocity = rpm / kIntakeGearRatio / kFalconTicksToRPM;
    m_masterMotor.set(ControlMode.Velocity, falconVelocity);
  }

  /**
   * Runs the intake front roller directly at a percentage of maximum output
   *
   * @param speedPercent Speed to run the motor: 0.0 (stop) to 1.0 (forward) or (-1.0) (reverse)
   */
  public void DEBUG_setMotorPercent(double speedPercent) {
    m_intakeMotorGroup.set(speedPercent);
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

    // Configure front roller motor
    m_masterMotor.setNeutralMode(NeutralMode.Coast);
    setPIDGains(m_PIDGains);
    m_masterMotor.config_IntegralZone(kPIDLoopIdx, kTimeoutMs);
    m_masterMotor.configPeakOutputForward(kMaxOutputPercent, kTimeoutMs);
    m_masterMotor.configPeakOutputReverse(-1.0 * kMaxOutputPercent, kTimeoutMs);
  }

  /**
   * Applies given closed-loop gains to intake motors
   *
   * @param gains Gains to apply
   */
  public void setPIDGains(PIDGains gains) {
    m_PIDGains = gains;
    final int timeoutMs = 10;
    m_masterMotor.config_kF(kPIDLoopIdx, kDefaultkFF, timeoutMs);
    m_masterMotor.config_kP(kPIDLoopIdx, gains.kP, timeoutMs);
    m_masterMotor.config_kI(kPIDLoopIdx, gains.kI, timeoutMs);
    m_masterMotor.config_kD(kPIDLoopIdx, gains.kD, timeoutMs);

    m_slaveMotor.config_kF(kPIDLoopIdx, kDefaultkFF, timeoutMs);
    m_slaveMotor.config_kP(kPIDLoopIdx, gains.kP, timeoutMs);
    m_slaveMotor.config_kI(kPIDLoopIdx, gains.kI, timeoutMs);
    m_slaveMotor.config_kD(kPIDLoopIdx, gains.kD, timeoutMs);
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
    // Gather telemetry from front roller motor
    telemetry.masterMotor.motorRPM = m_masterMotor.getSelectedSensorVelocity() * kFalconTicksToRPM;
    telemetry.masterMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.masterMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.masterMotor.temperatureCelcius = m_masterMotor.getTemperature();
    telemetry.slaveMotor.motorRPM = m_masterMotor.getSelectedSensorVelocity() * kFalconTicksToRPM;
    telemetry.slaveMotor.motorVolts = m_masterMotor.getMotorOutputVoltage();
    telemetry.slaveMotor.statorCurrentAmps = m_masterMotor.getStatorCurrent();
    telemetry.slaveMotor.temperatureCelcius = m_masterMotor.getTemperature();
  }
}
