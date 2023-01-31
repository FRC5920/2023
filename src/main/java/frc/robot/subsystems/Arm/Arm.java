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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final WPI_TalonFX ArmYMotorMaster =
      new WPI_TalonFX(Constants.ArmConstants.kArmYMotorMasterPort);

  private final WPI_TalonSRX HandBottomRoller =
      new WPI_TalonSRX(Constants.ArmConstants.kHandBottomRollerPort);
  private final WPI_TalonSRX HandTopFrontRoller =
      new WPI_TalonSRX(Constants.ArmConstants.kHandTopFrontRollerPort);
  private final WPI_TalonSRX HandTopBackRoller =
      new WPI_TalonSRX(Constants.ArmConstants.kHandTopBackRollerPort);
  // private final WPI_TalonFX ArmYMotorSlave = new
  // WPI_TalonFX(Constants.ArmConstants.kArmYMotorSlavePort);
  // private final WPI_TalonFX ArmExtender = new
  // WPI_TalonFX(Constants.ArmConstants.kArmExtenderPort);
  private final Pneumatics myPneumatics;
  private final double HandRollerSpeed = 0.5;

  public Arm(Pneumatics s_Pneumatics) {
    this.myPneumatics = s_Pneumatics;
    HandTopBackRoller.setNeutralMode(NeutralMode.Brake);
    HandTopFrontRoller.setNeutralMode(NeutralMode.Brake);
    HandBottomRoller.setNeutralMode(NeutralMode.Brake);
    ArmYMotorMaster.setNeutralMode(NeutralMode.Brake);
    //  ArmYMotorSlave.setNeutralMode(NeutralMode.Brake);
    //  ArmExtender.setNeutralMode(NeutralMode.Brake);
  }

  private void setArmPosition(int desiredPosition) {
    if (desiredPosition >= Constants.ArmConstants.kArmExtendedHigh) {
      myPneumatics.goingBackward();
    } else {
      myPneumatics.goingForward();
    }
    ;
    ArmYMotorMaster.setSelectedSensorPosition(desiredPosition);
  }

  private void spinHandRollers(boolean wantsConeNotCube, boolean intakingNotPlacing) {
    if (intakingNotPlacing == false) {}

    if (wantsConeNotCube == true) {}
  }

  public void armForward() {
    ArmYMotorMaster.set(ControlMode.PercentOutput, 0.2);
  }

  public void armBackward() {
    ArmYMotorMaster.set(ControlMode.PercentOutput, -0.2);
  }

  public void intakeOn() {
    HandBottomRoller.set(ControlMode.PercentOutput, 0.5);
    HandTopFrontRoller.set(ControlMode.PercentOutput, 0.5);
    HandTopBackRoller.set(ControlMode.PercentOutput, 0.5);
  }

  public void intakeOff() {
    HandBottomRoller.set(ControlMode.PercentOutput, 0);
    HandTopFrontRoller.set(ControlMode.PercentOutput, 0);
    HandTopBackRoller.set(ControlMode.PercentOutput, 0);
  }

  public void placeOn() {
    HandBottomRoller.set(ControlMode.PercentOutput, -0.5);
    HandTopFrontRoller.set(ControlMode.PercentOutput, -0.5);
    HandTopBackRoller.set(ControlMode.PercentOutput, -0.5);
  }

  public void placeOff() {
    HandBottomRoller.set(ControlMode.PercentOutput, 0);
    HandTopFrontRoller.set(ControlMode.PercentOutput, 0);
    HandTopBackRoller.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "ArmYMotor Encoder Value", (ArmYMotorMaster.getSelectedSensorPosition()));
  }
}
