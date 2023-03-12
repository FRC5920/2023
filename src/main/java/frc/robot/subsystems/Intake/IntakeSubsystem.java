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

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;

/** Subsystem for managing rollers used to pull in and shoot game pieces */
public class IntakeSubsystem extends SubsystemBase {
  /** Set this constant to true if the subsystem's dashboard tab should be displayed */
  private static boolean kEnableDashboardTab = true;

  // Motor CAN ID's
  public static final int kIntakeMasterMotorCANId = 40;
  public static final int kIntakeSlaveMotorCANId = 41;

  /** Gear ratio used to couple intake motor axles to rollers */
  public static final double kIntakeGearRatio = 1.0 / 3.33;

  /** Peak output (%) that intake motors should run */
  private static final double kMaxMotorOutputPercent = 0.75;
  /** Time required to ramp from neutral to full scale motor output */
  private static final double kOpenLoopRampSec = 0.5;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  /** Motors that drive the intake rollers */
  private final WPI_TalonFX m_motors[] =
      new WPI_TalonFX[] {
        new WPI_TalonFX(kIntakeMasterMotorCANId), new WPI_TalonFX(kIntakeSlaveMotorCANId)
      };

  private final WPI_TalonFX m_masterMotor = m_motors[0];
  private final WPI_TalonFX m_slaveMotor = m_motors[1];

  /** Dashboard tab for the intake subsystem */
  final IntakeDashboardTab m_dashboardTab;

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
  public void activatePreset(SpeedPreset preset) {
    setSpeedPercent(preset.motorSpeed);
  }

  /**
   * Sets the intake motor speed to a given RPM
   *
   * @param rpm Speed (percent of full scale output) to run the intake at
   * @remarks Negative values pull a game piece in; positive push it out.
   */
  public void setSpeedPercent(double percent) {
    // Convert RPM to falcon sensor velocity
    m_masterMotor.set(percent);
  }

  /** Deactivates intake motors */
  public void stopIntake() {
    setSpeedPercent(0.0);
  }

  /** Returns the present motor speed as a percentage of full scale output */
  public double getSpeedPercent() {
    return m_masterMotor.get();
  }

  /** Returns the present motor current reading in amps */
  public double getMotorCurrentAmps() {
    return m_masterMotor.getStatorCurrent();
  }

  /** Telemetry gathered from intake motors */
  public static class MotorTelemetry {
    /** Speed of the motor as a percentage of full scale (-1.0 to 1.0) */
    double speedPercent = 0.0;

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
    telemetry.speedPercent = motor.getSelectedSensorVelocity(kPIDLoopIdx);
    telemetry.motorVolts = motor.getMotorOutputVoltage();
    telemetry.statorCurrentAmps = motor.getStatorCurrent();
    telemetry.temperatureCelcius = motor.getTemperature();
  }

  /** Registers the subsystem's dashboard tab with the dashboard subsystem */
  public void registerDashboardTab(DashboardSubsystem dashboardSubsystem) {
    if (kEnableDashboardTab) {
      dashboardSubsystem.add(m_dashboardTab);
    }
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
    for (WPI_TalonFX motor : m_motors) {
      // Configure neutral deadband to be tighter than the default
      motor.configNeutralDeadband(0.001);

      // Configure peak and nominal outputs
      motor.configNominalOutputForward(0, kTimeoutMs);
      motor.configNominalOutputReverse(0, kTimeoutMs);
      motor.configPeakOutputForward(kMaxMotorOutputPercent, kTimeoutMs);
      motor.configPeakOutputReverse(-1.0 * kMaxMotorOutputPercent, kTimeoutMs);

      // Configure the motors to ramp up to speed.  This eliminates the initial inrush current spike
      // that occurs as the motor starts up and has to overcome the inertia of itself and the
      // rollers
      motor.configOpenloopRamp(kOpenLoopRampSec);
    }
  }

  /** Called by the scheduler to service the subsystem */
  @Override
  public void periodic() {}
}
