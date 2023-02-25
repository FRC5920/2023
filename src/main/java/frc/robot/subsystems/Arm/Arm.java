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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pneumatics.Pneumatics;

public class Arm extends SubsystemBase {
  // TODO: Testing
  public XboxController opXboxController = new XboxController(1);
  /** Creates a new Arm. */
  private static final WPI_TalonFX ArmYMotorMaster =
      new WPI_TalonFX(Constants.ArmConstants.kArmYMotorMasterPort);

  private static final WPI_TalonFX HandFrontRoller =
      new WPI_TalonFX(Constants.ArmConstants.kHandFrontRollerPort);
  // private static final CANSparkMax HandBackRoller =
  //   new CANSparkMax(Constants.ArmConstants.kHandBackRollerPort, MotorType.kBrushless);
  private static final WPI_TalonFX ArmYMotorSlave =
      new WPI_TalonFX(Constants.ArmConstants.kArmYMotorSlavePort);

  private static final WPI_TalonFX ArmExtender =
      new WPI_TalonFX(Constants.ArmConstants.kArmExtenderPort);

  private static Pneumatics myPneumatics;
  private static final double HandRollerSpeed = 0.5;

  public enum GamePieceType {
    Cone,
    Cube
  }

  public enum DoWhatWithGamePiece {
    In,
    Out
  }

  public enum Rank {
    PickUp(0),
    InRobot(200),
    Low(50),
    Medium(2000),
    High(3000);

    private final int encoderCount;

    private Rank(int count) {
      this.encoderCount = count;
    }

    public int getEncoderCount() {
      return encoderCount;
    }
  }

  public enum ArmExtenderPosition {
    // public static final int[] tRexPosition = new int []{0, 13629, 31880, 43589, 54841, 68187};

    StowedAway(0),
    /** Arm is fully retracted */
    OnFloor(1000),
    /** Arm is down on the floor */
    MiddleRank(2000),
    /** Arm is reaching to the middle rank */
    TopRank(3000);
    /** Arm is reaching to the top rank */
    private final int encoderCount;

    private ArmExtenderPosition(int count) {
      this.encoderCount = count;
    }

    public int getEncoderCount() {
      return encoderCount;
    }
  };

  public Arm(Pneumatics s_Pneumatics) {
    ArmYMotorSlave.follow(ArmYMotorMaster);
    ArmYMotorSlave.setInverted(true);
    this.myPneumatics = s_Pneumatics;
    configurePID();
  }

  private void configurePID() {
    // set ArmExtender PID coefficients
    ArmExtender.config_kF(
        Constants.ArmConstants.kArmExtenderPIDLoopIdx,
        Constants.ArmConstants.kArmExtenderFF,
        Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.config_kP(
        Constants.ArmConstants.kArmExtenderPIDLoopIdx,
        Constants.ArmConstants.kArmExtenderP,
        Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.config_kI(
        Constants.ArmConstants.kArmExtenderPIDLoopIdx,
        Constants.ArmConstants.kArmExtenderI,
        Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.config_kD(
        Constants.ArmConstants.kArmExtenderPIDLoopIdx,
        Constants.ArmConstants.kArmExtenderD,
        Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.config_IntegralZone(
        Constants.ArmConstants.kArmExtenderPIDLoopIdx,
        Constants.ArmConstants.kArmExtenderIz,
        Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.configNominalOutputForward(0, Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.configNominalOutputReverse(0, Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.configPeakOutputForward(1, Constants.ArmConstants.kArmExtenderTimeoutMs);
    ArmExtender.configPeakOutputReverse(-1, Constants.ArmConstants.kArmExtenderTimeoutMs);

    // set ArmYMotorMaster PID coefficients
    ArmYMotorMaster.config_kF(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYFF,
        Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.config_kP(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYP,
        Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.config_kI(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYI,
        Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.config_kD(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYD,
        Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.config_IntegralZone(
        Constants.ArmConstants.kArmYPIDLoopIdx,
        Constants.ArmConstants.kArmYIz,
        Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.configNominalOutputForward(0, Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.configNominalOutputReverse(0, Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.configPeakOutputForward(1, Constants.ArmConstants.kArmYTimeoutMs);
    ArmYMotorMaster.configPeakOutputReverse(-1, Constants.ArmConstants.kArmYTimeoutMs);
  }

  public static void setArmExtension(ArmExtenderPosition extensionEncoderValue) {
    ArmExtender.set(TalonFXControlMode.Position, extensionEncoderValue.encoderCount);
  }

  public static void setArmPosition(int desiredPosition) {
    if (desiredPosition >= Constants.ArmConstants.kArmExtendedHigh) {
      myPneumatics.goingBackward();
    } else {
      myPneumatics.goingForward();
    }
    ;
    ArmYMotorMaster.set(TalonFXControlMode.Position, desiredPosition);
  }

  public static void spinAllHandRollers(
      GamePieceType pickUpWhat, DoWhatWithGamePiece desiredHandAction) {
    double HandFrontRollerSpeedPercent = HandRollerSpeed;
    double HandBackRollerSpeedPercent = 0.0;
    switch (pickUpWhat) {
      case Cone:
        HandBackRollerSpeedPercent = HandRollerSpeed;
        break;
      case Cube:
        HandBackRollerSpeedPercent = -1 * HandRollerSpeed;
        break;
      default:
        break;
    }
    if (desiredHandAction == DoWhatWithGamePiece.Out) {
      HandFrontRollerSpeedPercent *= -1;
      HandBackRollerSpeedPercent *= -1;
    }
    HandFrontRoller.set(HandFrontRollerSpeedPercent);
    // HandBackRoller.set(HandBackRollerSpeedPercent);
  }

  public static void zeroHandRollers() {
    // HandBackRoller.set(0);
    HandFrontRoller.set(0);
  }
  ;

  public void armForward() {
    ArmYMotorMaster.set(ControlMode.PercentOutput, 1);
  }

  public void armBackward() {
    ArmYMotorMaster.set(ControlMode.PercentOutput, -1);
  }

  public void intake() {
    HandFrontRoller.set(0.5);
    // HandBackRoller.set(0.5);
  }

  public void place() {
    HandFrontRoller.set(-0.5);
    // HandBackRoller.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: Testing Code
    ArmYMotorMaster.set(
        TalonFXControlMode.PercentOutput, opXboxController.getLeftX(), null, HandRollerSpeed);
    ArmExtender.set(TalonFXControlMode.PercentOutput, opXboxController.getRightY());
    //

    SmartDashboard.putNumber(
        "ArmYMotor Encoder Value", (ArmYMotorMaster.getSelectedSensorPosition()));
  }
}
