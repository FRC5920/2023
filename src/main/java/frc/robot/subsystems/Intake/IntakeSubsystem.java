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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePieceType;

/** Subsystem for managing the cargo intake at the end of the robot arm */
public class IntakeSubsystem extends SubsystemBase {
  static final int kFrontRollerMotorCANId = Constants.ArmConstants.kHandFrontRollerPort;
  static final int kRearRollerMotorCANId = Constants.ArmConstants.kHandBackRollerPort;

  // FalconFX reports velocity in counts per 100ms
  // 1 revolution = 2048 counts
  // 1 minutes = 60 * 10 * 100ms
  // conversion is  600  / 2048
  private final double kFalconTicks2RPm = 600.0 / 2048.0;

  /** Motor speeds used to disable the intake motors [front, rear] */
  static final int kDisabledMotorRPMs[] = new int[] {0, 0};

  /** Motor speeds used to acquire a cone [front, rear] */
  static final int kAcquireConeMotorRPMs[] = new int[] {0, 0};

  /** Motor speeds used to acquire a cube [front, rear] */
  static final int kAcquireCubeMotorRPMs[] = new int[] {0, 0};

  /** Motor speeds used to place a cone [front, rear] */
  static final int kPlaceConeMotorRPMs[] = new int[] {0, 0};

  /** Motor speeds used to place a cube [front, rear] */
  static final int kPlaceCubeMotorRPMs[] = new int[] {0, 0};

  /** Front roller PID coefficients */
  private double kFrontPID_FF = 0.055;

  private double kFrontPID_P = 0.22;
  private double kFrontPID_I = 0.002;
  private double kFrontPID_D = 0;
  private double kFrontPID_Iz = 100;

  /** Motor that drives the front intake roller */
  WPI_TalonFX m_frontRollerMotor;

  /** Motor that drives the rear intake roller */
  private final CANSparkMax m_rearRollerMotor =
      new CANSparkMax(kRearRollerMotorCANId, MotorType.kBrushless);

  /** Motor measurements */
  final IntakeSubsystemTelemetry m_telemetry = new IntakeSubsystemTelemetry();

  /** Dashboard tab for the intake subsystem */
  final IntakeDashboardTab m_dashboardTab = new IntakeDashboardTab(this, m_telemetry);

  /** Creates an instance of the subsystem */
  public IntakeSubsystem() {
    // TODO: add motor safety
    m_frontRollerMotor = new WPI_TalonFX(kFrontRollerMotorCANId);
    configure();
  }

  private void acquire(GamePieceType gamePiece) {
    int rollerRPM[];

    switch (gamePiece) {
      case Cone:
        rollerRPM = kAcquireConeMotorRPMs;
        break;
      case Cube:
        rollerRPM = kAcquireCubeMotorRPMs;
        break;
      default:
        rollerRPM = kDisabledMotorRPMs;
        break;
    }

    setFrontRollerRPM(rollerRPM[MotorID.frontRoller.index]);
    setRearRollerRPM(rollerRPM[MotorID.rearRoller.index]);
  }

  private void place(GamePieceType gamePiece) {
    int rollerRPM[];

    switch (gamePiece) {
      case Cone:
        rollerRPM = kPlaceConeMotorRPMs;
        break;
      case Cube:
        rollerRPM = kPlaceCubeMotorRPMs;
        break;
      default:
        rollerRPM = kDisabledMotorRPMs;
        break;
    }

    setFrontRollerRPM(rollerRPM[MotorID.frontRoller.index]);
    setRearRollerRPM(rollerRPM[MotorID.rearRoller.index]);
  }

  public void spinAllHandRollers(GamePieceType gamePiece, IntakeAction action) {
    switch (action) {
      case Acquire:
        acquire(gamePiece);
        break;

      case Place:
        place(gamePiece);
        break;

      case Hold:
        // TODO: what do we need to do to hold a game piece?
        break;

      case TurnOff:
        acquire(GamePieceType.Nothing);
        break;
    }
  }

  public void shutoff() {
    setFrontRollerRPM(kDisabledMotorRPMs[MotorID.frontRoller.index]);
    setRearRollerRPM(kDisabledMotorRPMs[MotorID.rearRoller.index]);
  }

  /** Called by the scheduler to service the subsystem */
  @Override
  public void periodic() {
    // Gather telemetry from front roller motor
    m_telemetry.frontMotor.motorRPM =
        m_frontRollerMotor.getSelectedSensorVelocity() * kFalconTicks2RPm;
    m_telemetry.frontMotor.motorVolts = m_frontRollerMotor.getMotorOutputVoltage();
    m_telemetry.frontMotor.statorCurrentAmps = m_frontRollerMotor.getStatorCurrent();
    m_telemetry.frontMotor.temperatureCelcius = m_frontRollerMotor.getTemperature();

    // TODO: gather telemetry from rear roller motor
  }

  /** Sets the speed of the front roller in RPM */
  public void setFrontRollerRPM(double rpm) {
    m_frontRollerMotor.set(TalonFXControlMode.Velocity, rpm);
  }

  /** Sets the speed of the rear roller in RPM */
  public void setRearRollerRPM(double rpm) {
    // TODO: set the target velocity of the rear roller motor
//https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

  }

  /**
   * Runs the intake front roller
   *
   * @param speedPercent Speed to run the motor: 0.0 (stop) to 1.0 (forward) or (-1.0) (reverse)
   */
  public void DEBUG_runFrontRoller(double speedPercent) {
    m_frontRollerMotor.set(ControlMode.PercentOutput, speedPercent);
  }

  /**
   * Runs the intake rear roller
   *
   * @param speedPercent Speed to run the motor: 0.0 (stop) to 1.0 (forward) or (-1.0) (reverse)
   */
  public void DEBUG_runRearRoller(double speedPercent) {
    // TODO: run the rear roller motor
    m_rearRollerMotor.set(speedPercent);
  }

  /** Returns the subsystem's dashboard tab */
  public IntakeDashboardTab getDashboardTab() {
    return m_dashboardTab;
  }

  /** Configure motors in the subsystem */
  private void configure() {
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    // Configure front roller motor
    m_frontRollerMotor.setNeutralMode(NeutralMode.Brake);
    m_frontRollerMotor.config_kF(kPIDLoopIdx, kFrontPID_FF, kTimeoutMs);
    m_frontRollerMotor.config_kP(kPIDLoopIdx, kFrontPID_P, kTimeoutMs);
    m_frontRollerMotor.config_kI(kPIDLoopIdx, kFrontPID_I, kTimeoutMs);
    m_frontRollerMotor.config_kD(kPIDLoopIdx, kFrontPID_D, kTimeoutMs);
    m_frontRollerMotor.config_IntegralZone(kPIDLoopIdx, kFrontPID_Iz, kTimeoutMs);
    // m_frontRollerMotor.configNominalOutputForward(0, kTimeoutMs);
    // m_frontRollerMotor.configNominalOutputReverse(0, kTimeoutMs);
    // m_frontRollerMotor.configPeakOutputForward(1, kTimeoutMs);
    // m_frontRollerMotor.configPeakOutputReverse(-1, kTimeoutMs);
  }

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
    final MotorTelemetry frontMotor = new MotorTelemetry();
    final MotorTelemetry rearMotor = new MotorTelemetry();
  }

  private enum MotorID {
    frontRoller(0),
    rearRoller(1);

    private int index;

    private MotorID(int idx) {
      index = idx;
    }
  }

  /** An enumeration of actions that can be taken on a game piece */
  public enum IntakeAction {
    /** Turn off the intake motors */
    TurnOff,

    /** Acquire a game piece */
    Acquire,

    /** Place a game piece */
    Place,

    /** Hold a game piece */
    Hold
  }
}
