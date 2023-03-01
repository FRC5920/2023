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
package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pneumatics.Pneumatics.WristPosition;

public class Arm extends SubsystemBase {
  static final int kAngleMotorCANId = Constants.ArmConstants.kArmYMotorMasterPort;
  static final int kExtenderMotorCANId = Constants.ArmConstants.kArmExtenderPort;

  /** Number of encoder counts per rotation */
  static final int kFalconCountsPerRotation = 2048;

  /** Diameter (meters) of the angle motor gear TODO: measure this value */
  static final double kAngleMotorCouplingDiameterMeters = Units.inchesToMeters(4);

  /** Gear ratio used to couple the angle motor to the arm mechanism */
  static final double kAngleMotorGearRatio = 1/150; // TODO: set this constant

  /** Diameter (meters) of the angle motor gear TODO: measure this value */
  static final double kExtenderMotorCouplingDiameterMeters = Units.inchesToMeters(2);

  /** Gear ratio used to couple the elevator motor to the elevator mechanism */
  static final double kExtenderMotorGearRatio = 1.0; // TODO: set this constant

  /** Motor controlling the angle of the arm */
  private final WPI_TalonFX m_angleMotor = new WPI_TalonFX(kAngleMotorCANId);

  /** Motor controlling the extension of the arm */
  private final WPI_TalonFX m_extenderMotor = new WPI_TalonFX(kExtenderMotorCANId);

  /** Dashboard tab for the Arm subsystem */
  private final ArmDashboardTab m_dashboardTab;

  /** Output values displayed on the dashboard */
  private final ArmSubsystemTelemetry m_telemetry = new ArmSubsystemTelemetry();

  public enum Rank {
    PickUp,
    Low,
    Medium,
    High
  }

  /** Creates an instance of the subsystem */
  public Arm() {
    configureMotors();
    m_dashboardTab = new ArmDashboardTab(m_telemetry);
  }

  public void setAngle(AnglePreset preset) {
    // NOTE: we may need to apply a trapezoidal motion profile here
    m_angleMotor.setSelectedSensorPosition(preset.encoderCount);
  }

  public void armForward(double percentOutput) {
    m_angleMotor.set(ControlMode.PercentOutput, 1);
  }

  public void armBackward(double percentOutput) {
    m_angleMotor.set(ControlMode.PercentOutput, -1);
  }

  /**
   * Debug routine to set the angle motor position in closed-loop mode
   *
   * @param position Position to set the motor to in falcon encoder ticks
   */
  public void DEBUG_setAnglePosition(double position) {
    m_angleMotor.set(TalonFXControlMode.Position, position);
  }

  /**
   * Debug routine to set the extender motor position in closed-loop mode
   *
   * @param position Position to set the motor to in falcon encoder ticks
   */
  public void DEBUG_setExtenderPosition(double position) {
    m_extenderMotor.set(TalonFXControlMode.Position, position);
  }

  /**
   * Debug routine to drive the angle motor directly in open-loop mode
   *
   * @param speedPercent Percentage of max output applied to the motor (0.0 to +/-1.0)
   */
  public void DEBUG_runAngleMotor(double speedPercent) {
    m_angleMotor.set(speedPercent);
  }

  /**
   * Debug routine to drive the extender motor directly in open-loop mode
   *
   * @param speedPercent Percentage of max output applied to the motor (0.0 to +/-1.0)
   */
  public void DEBUG_runExtenderMotor(double speedPercent) {
    m_extenderMotor.set(speedPercent);
  }

  /** Converts an encoder value from the angle motor to a corresponding angle in degrees */
  private static double angleEncoderToDegrees(double encoderCount) {
    // TODO: verify this calculation
    return Units.rotationsToDegrees(
        encoderCount / (double) kFalconCountsPerRotation * kAngleMotorGearRatio);
  }

  /**
   * Converts an encoder value from the extender motor to a corresponding extender distance in
   * meters
   */
  private static double extenderEncoderToMeters(double encoderCount) {
    // TODO: verify this calculation
    final double kEncoderDistancePerPulse =
        (kExtenderMotorCouplingDiameterMeters * Math.PI)
            / (double) kFalconCountsPerRotation
            * kAngleMotorGearRatio;
    return encoderCount * kEncoderDistancePerPulse;
  }

  @Override
  public void periodic() {
    // Gather telemetry for angle motor
    m_telemetry.angleTelemetry.motorVolts = m_angleMotor.getMotorOutputVoltage();
    m_telemetry.angleTelemetry.statorCurrentAmps = m_angleMotor.getStatorCurrent();
    m_telemetry.angleTelemetry.temperatureCelcius = m_angleMotor.getTemperature();
    m_telemetry.angleEncoderCount = m_angleMotor.getSelectedSensorPosition();
    m_telemetry.angleDegrees = angleEncoderToDegrees(m_telemetry.angleEncoderCount);

    // Gather telemetry for extender motor
    m_telemetry.extenderTelemetry.motorVolts = m_extenderMotor.getMotorOutputVoltage();
    m_telemetry.extenderTelemetry.statorCurrentAmps = m_extenderMotor.getStatorCurrent();
    m_telemetry.extenderTelemetry.temperatureCelcius = m_extenderMotor.getTemperature();
    m_telemetry.extenderEncoderCount = m_extenderMotor.getSelectedSensorPosition();
    m_telemetry.extenderPositionMeters = extenderEncoderToMeters(m_telemetry.extenderEncoderCount);
  }

  /** Returns the subsystem's dashboard tab */
  public ArmDashboardTab getDashboardTab() {
    return m_dashboardTab;
  }

  /** Configures motors in the subsystem */
  private void configureMotors() {
    // set Arm angle settings
    m_angleMotor.setNeutralMode(NeutralMode.Brake);
    // set ArmExtender PID coefficients
    m_extenderMotor.config_kF(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYFF,
        Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.config_kP(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYP,
        Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.config_kI(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYI,
        Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.config_kD(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYD,
        Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.config_IntegralZone(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYIz,
        Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.configNominalOutputForward(0, Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.configNominalOutputReverse(0, Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.configPeakOutputForward(1, Constants.ArmConstants.kArmYTimeoutMs);
    m_extenderMotor.configPeakOutputReverse(-1, Constants.ArmConstants.kArmYTimeoutMs);
  }

  /** An enumeration of cargo types */
  public enum AnglePreset {
    /** Default angle */
    kDefault(0), // TODO: establish this encoder count experimentally

    /** Angle used when intaking cargo */
    kIntake(0), // TODO: establish this encoder count experimentally

    /** Angle used when placing cargo */
    kPlacement(0); // TODO: establish this encoder count experimentally

    /** The encoder count corresponding to a given preset */
    private int encoderCount;

    private AnglePreset(int count) {
      encoderCount = count;
    }

    /** Get the human-readable name of the angle preset */
    @Override
    public String toString() {
      return this.name();
    }
  }

  public static class MotorTelemetry {
    /** Voltage applied to the motor (Volts) */
    double motorVolts = 0.0;

    /** Motor current measurement (Amperes) from the motor */
    double statorCurrentAmps = 0.0;

    /** Motor temperature measurement (Celcius) from the motor */
    double temperatureCelcius = 0.0;
  }

  /** Measurements gathered in the Arm subsystem */
  public class ArmSubsystemTelemetry {
    /** Angle motor measurements */
    MotorTelemetry angleTelemetry = new MotorTelemetry();
    /** Raw encoder count from the angle motor */
    double angleEncoderCount = 0;
    /** Angle motor reading in degrees */
    double angleDegrees = 0.0;

    /** Extender motor telemetry */
    MotorTelemetry extenderTelemetry = new MotorTelemetry();
    /** Raw encoder count from the extender motor */
    double extenderEncoderCount = 0;
    /** Extender motor position in meters */
    double extenderPositionMeters = 0.0;

    /** true when the wrist is in the inverted position; else false */
    WristPosition wristPosition = WristPosition.Normal;
  }
}
