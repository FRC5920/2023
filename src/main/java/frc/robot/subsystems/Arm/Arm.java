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
import frc.robot.subsystems.Arm.Pneumatics.WristPosition;

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
  private final ArmDashboardTab m_armDashboardTab;
  /** Output values displayed on the dashboard */
  private final ArmSubsystemDashboardOutputs m_dashboardOutputs =
      new ArmSubsystemDashboardOutputs();
  /** History of input values received from the dashboard: 0=current; 1=previous */
  private final ArmSubsystemDashboardInputs m_dashboardInputs = new ArmSubsystemDashboardInputs();

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
    WristPosition wristPosition = WristPosition.Normal;

    /** Motor current measurement (Amperes) from the top intake motor */
    double upperIntakeCurrentAmps = 0.0;

    /** Motor current measurement (Amperes) from the bottom intake motor */
    double lowerIntakeCurrentAmps = 0.0;
  }

  public class ArmSubsystemDashboardInputs {
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
    m_armDashboardTab = new ArmDashboardTab(m_dashboardOutputs);
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

  public void runAngleMotor(double percent) {
    ArmYMotorMaster.set(percent * 0.2); // Set max output to 20% for slow motion
  }

  public void runExtenderMotor(double percent) {
    ArmExtender.set(percent * 0.2); // Set max output to 20% for slow motion
  }

  public void runIntakeMotor(double percent) {
    double motorSpeedPercent = 0.0;
    if (percent > 0) {
      motorSpeedPercent = m_dashboardInputs.intakeCargoMotorSpeed;
    } else if (percent < 0) {
      motorSpeedPercent = m_dashboardInputs.placeCargoMotorSpeed;
    }

    HandBottomRoller.set(motorSpeedPercent);
    HandTopBackRoller.set(motorSpeedPercent);
  }

  public void toggleWristPosition() {
    Pneumatics.WristPosition position =
        myPneumatics.getWristPosition() == WristPosition.Normal
            ? WristPosition.Inverted
            : WristPosition.Normal;
    myPneumatics.setWristPosition(position);
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
    double intakeSpeed = m_dashboardInputs.intakeCargoMotorSpeed;
    double HandBottomRollerSpeedPercent = intakeSpeed;
    double HandTopBackRollerSpeedPercent = 0.0;
    switch (pickUpWhat) {
      case Cone:
        HandTopBackRollerSpeedPercent = intakeSpeed;
        break;
      case Cube:
        HandTopBackRollerSpeedPercent = -1 * intakeSpeed;
        break;
      default:
        break;
    }
    if (desiredHandAction == DoWhatWithGamePiece.Out) {
      double placementSpeed = m_dashboardInputs.placeCargoMotorSpeed;
      HandBottomRollerSpeedPercent = -1 * placementSpeed;
      HandTopBackRollerSpeedPercent = -1 * placementSpeed;
    }

    HandBottomRoller.set(HandBottomRollerSpeedPercent);
    HandTopBackRoller.set(HandTopBackRollerSpeedPercent);
  }

  public void zeroHandRollers() {
    HandTopBackRoller.set(0);
    HandBottomRoller.set(0);
  }

  public void armForward(double percentOutput) {
    ArmYMotorMaster.set(ControlMode.PercentOutput, 1);
  }

  public void armBackward(double percentOutput) {
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

  public void initDashboard() {
    m_armDashboardTab.initialize();
  }

  @Override
  public void periodic() {
    // Update measurements
    m_dashboardOutputs.angleEncoderCount = ArmYMotorMaster.getSelectedSensorPosition();
    m_dashboardOutputs.angleDegrees = angleEncoderToDegrees(m_dashboardOutputs.angleEncoderCount);
    m_dashboardOutputs.extenderEncoderCount = ArmExtender.getSelectedSensorPosition();
    m_dashboardOutputs.extenderPositionMeters =
        extenderEncoderToDegrees(m_dashboardOutputs.extenderEncoderCount);
    m_dashboardOutputs.wristPosition = myPneumatics.getWristPosition();
    m_dashboardOutputs.lowerIntakeCurrentAmps = HandTopBackRoller.getOutputCurrent();
    m_dashboardOutputs.upperIntakeCurrentAmps = HandBottomRoller.getOutputCurrent();

    // Update dashboard inputs and outputs
    m_armDashboardTab.update(m_dashboardInputs);
  }
}
