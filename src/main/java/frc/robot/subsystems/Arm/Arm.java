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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final WPI_TalonFX ArmYMotorMaster =
      new WPI_TalonFX(Constants.ArmConstants.kArmYMotorMasterPort);

  private final CANSparkMax HandBottomRoller =
      new CANSparkMax(Constants.ArmConstants.kHandBottomRollerPort, MotorType.kBrushless);
  private final CANSparkMax HandTopBackRoller =
      new CANSparkMax(Constants.ArmConstants.kHandTopBackRollerPort, MotorType.kBrushless);
  // private final WPI_TalonFX ArmYMotorSlave = new
  // WPI_TalonFX(Constants.ArmConstants.kArmYMotorSlavePort);
  private final WPI_TalonFX ArmExtender = new WPI_TalonFX(Constants.ArmConstants.kArmExtenderPort);
  private final Pneumatics myPneumatics;
  private final double HandRollerSpeed = 0.5;

  /** Dashboard tab for the Arm subsystem */
  private final ArmDashboardTab m_ArmDashboardTab;
  /** Output values displayed on the dashboard */
  private final ArmSubsystemDashboardOutputs m_dashboardOutputs =
      new ArmSubsystemDashboardOutputs();
  /** History of input values received from the dashboard: 0=current; 1=previous */
  private final ArmSubsystemDashboardInputs m_dashboardInputHistory[] =
      new ArmSubsystemDashboardInputs[2];

  /** Measurements gathered in the Arm subsystem */
  public class ArmSubsystemDashboardOutputs {
    /** Raw encoder count from the angle motor */
    double angleEncoderCount = 0;
    /** Angle motor reading in degrees */
    double angleDegrees = 0.0;

    /** Raw encoder count from the extender motor */
    double extenderEncoderCount = 0;
    /** Extender motor position in meters */
    double extenderPositionMeters = 0.0;

    /** true when the wrist is in the inverted position; else false */
    boolean wristIsInverted = false;

    /** Motor current measurement (Amperes) from the top intake motor */
    double upperIntakeCurrentAmps = 0.0;

    /** Motor current measurement (Amperes) from the bottom intake motor */
    double lowerIntakeCurrentAmps = 0.0;
  }

  public class ArmSubsystemDashboardInputs {
    /** Dashboard sets this to true to signal that the angle encoder should be reset */
    boolean zeroAngleEncoder = false;

    /** Dashboard sets this to true to signal that the extender encoder should be reset */
    boolean zeroExtenderEncoder = false;

    /** Gives the speed to run intake motors at when taking in cargo */
    double intakeCargoMotorSpeed = 0.0;

    /** Gives the speed to run intake motors at when placing cargo */
    double placeCargoMotorSpeed = 0.0;
  }

  public enum GamePieceType {
    Cone,
    Cube
  }

  public enum DoWhatWithGamePiece {
    In,
    Out
  }

  public enum Rank {
    PickUp,
    Low,
    Medium,
    High
  }

  public Arm(Pneumatics s_Pneumatics) {
    this.myPneumatics = s_Pneumatics;
    configurePID();
    m_ArmDashboardTab = new ArmDashboardTab(m_dashboardOutputs);
  }

  private void configurePID() {
    // set ArmExtender PID coefficients
    ArmExtender.config_kF(
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kFF,
        Constants.ArmConstants.kTimeoutMs);
    ArmExtender.config_kP(
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kP,
        Constants.ArmConstants.kTimeoutMs);
    ArmExtender.config_kI(
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kI,
        Constants.ArmConstants.kTimeoutMs);
    ArmExtender.config_kD(
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kD,
        Constants.ArmConstants.kTimeoutMs);
    ArmExtender.config_IntegralZone(
        Constants.ArmConstants.kPIDLoopIdx,
        Constants.ArmConstants.kIz,
        Constants.ArmConstants.kTimeoutMs);
    ArmExtender.configNominalOutputForward(0, Constants.ArmConstants.kTimeoutMs);
    ArmExtender.configNominalOutputReverse(0, Constants.ArmConstants.kTimeoutMs);
    ArmExtender.configPeakOutputForward(1, Constants.ArmConstants.kTimeoutMs);
    ArmExtender.configPeakOutputReverse(-1, Constants.ArmConstants.kTimeoutMs);
  }

  private void setArmExtension(int extensionEncoderValue) {
    // TODO: Set up PID control here
  }

  public void setArmPosition(int desiredPosition) {
    if (desiredPosition >= Constants.ArmConstants.kArmExtendedHigh) {
      myPneumatics.goingBackward();
    } else {
      myPneumatics.goingForward();
    }
    ;
    ArmYMotorMaster.setSelectedSensorPosition(desiredPosition);
  }

  public void spinAllHandRollers(GamePieceType pickUpWhat, DoWhatWithGamePiece desiredHandAction) {
    double HandBottomRollerSpeedPercent = HandRollerSpeed;
    double HandTopBackRollerSpeedPercent = 0.0;
    switch (pickUpWhat) {
      case Cone:
        HandTopBackRollerSpeedPercent = HandRollerSpeed;
        break;
      case Cube:
        HandTopBackRollerSpeedPercent = -1 * HandRollerSpeed;
        break;
      default:
        break;
    }
    if (desiredHandAction == DoWhatWithGamePiece.Out) {
      HandBottomRollerSpeedPercent *= -1;
      HandTopBackRollerSpeedPercent *= -1;
    }
    HandBottomRoller.set(HandBottomRollerSpeedPercent);
    HandTopBackRoller.set(HandTopBackRollerSpeedPercent);
  }

  public void zeroHandRollers() {
    HandTopBackRoller.set(0);
    HandBottomRoller.set(0);
  }

  public void armForward() {
    ArmYMotorMaster.set(ControlMode.PercentOutput, 1);
  }

  public void armBackward() {
    ArmYMotorMaster.set(ControlMode.PercentOutput, -1);
  }

  public void intake() {
    HandBottomRoller.set(0.5);
    HandTopBackRoller.set(0.5);
  }

  public void place() {
    HandBottomRoller.set(-0.5);
    HandTopBackRoller.set(-0.5);
  }

  private double angleEncoderToDegrees(double encoderCount) {
    // TODO: convert encoder count to degrees
    return encoderCount;
  }

  private double extenderEncoderToDegrees(double encoderCount) {
    // TODO: convert encoder count to degrees
    return encoderCount;
  }

  @Override
  public void periodic() {
    // Update measurements
    m_dashboardOutputs.angleEncoderCount = ArmYMotorMaster.getSelectedSensorPosition();
    m_dashboardOutputs.angleDegrees = angleEncoderToDegrees(m_dashboardOutputs.angleEncoderCount);
    m_dashboardOutputs.extenderEncoderCount = ArmExtender.getSelectedSensorPosition();
    m_dashboardOutputs.extenderPositionMeters =
        extenderEncoderToDegrees(m_dashboardOutputs.extenderEncoderCount);
    m_dashboardOutputs.wristIsInverted =
        (myPneumatics.getWristPosition() == Pneumatics.WristPosition.Inverted);
    m_dashboardOutputs.lowerIntakeCurrentAmps = HandTopBackRoller.getOutputCurrent();
    m_dashboardOutputs.upperIntakeCurrentAmps = HandBottomRoller.getOutputCurrent();

    // Update dashboard inputs and outputs
    m_ArmDashboardTab.update(m_dashboardInputHistory[0]);

    // Process dashboard inputs
    if (m_dashboardInputHistory[0].zeroAngleEncoder
        != m_dashboardInputHistory[1].zeroAngleEncoder) {
      // TODO: reset the encoder count
      m_dashboardOutputs.angleEncoderCount += 1000;
    }

    // Rotate dashboard input history: element 0 --> element 1, element 1 --> element 0
    ArmSubsystemDashboardInputs temp = m_dashboardInputHistory[1];
    m_dashboardInputHistory[1] = m_dashboardInputHistory[0];
    m_dashboardInputHistory[0] = temp;
  }
}
