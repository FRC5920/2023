////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5290 - Vikotics    **                       |
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  ;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "ArmYMotor Encoder Value", (ArmYMotorMaster.getSelectedSensorPosition()));
  }
}
