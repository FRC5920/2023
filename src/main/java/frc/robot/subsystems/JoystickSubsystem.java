////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
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
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Joystick.AxisProcChain;
import frc.lib.Joystick.ProcessedXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.SnapToGrid;
import frc.robot.commands.Arm.ContinuousGetObject;
import frc.robot.commands.Arm.PlaceObject;
import frc.robot.commands.Arm.RotateIntake;
import frc.robot.subsystems.Arm.Arm;

/** A subsystem providing/managing Xbox controllers for driving the robot manually */
public class JoystickSubsystem extends SubsystemBase {

  public enum ControllerId {
    kDriver(0),
    kOperator(1);

    public final int port;

    ControllerId(int portNum) {
      this.port = portNum;
    }
  }

  // ---------- Joystick Processing Constants -----------------
  public static final double kDriverLeftStickSensitivity = 0.3;
  public static final double kDriverLeftStickDeadbands[] = {0.1, 0.95};
  public static final double kDriverRightStickSensitivity = 0.3;
  public static final double kDriverRightStickDeadbands[] = {0.1, 0.95};
  public static final double kDriverTriggerSensitivity = 1.0;
  public static final double kDriverTriggerDeadbands[] = {0.1, 0.95};

  public static final double kOperatorLeftStickSensitivity = 0.3;
  public static final double kOperatorLeftStickDeadbands[] = {0.1, 0.95};
  public static final double kOperatorRightStickSensitivity = 0.3;
  public static final double kOperatorRightStickDeadbands[] = {0.1, 0.95};
  public static final double kOperatorTriggerSensitivity = 1.0;
  public static final double kOperatorTriggerDeadbands[] = {0.1, 0.95};

  /** Xbox controller used by the robot driver */
  public ProcessedXboxController driverController;

  /** Xbox controller used by the robot operator */
  public ProcessedXboxController operatorController;

  /** Button */
  /** Creates a new JoystickSubsystem */
  public JoystickSubsystem() {
    // Configure driver controller stick and trigger processing
    driverController = new ProcessedXboxController(ControllerId.kDriver.port);
    AxisProcChain.Config stickConfig =
        new AxisProcChain.Config(kDriverLeftStickSensitivity, kDriverLeftStickDeadbands);
    driverController.getStickProcessing(XboxController.Axis.kLeftX).configure(stickConfig);
    driverController.getStickProcessing(XboxController.Axis.kLeftY).configure(stickConfig);
    stickConfig =
        new AxisProcChain.Config(kDriverRightStickSensitivity, kDriverRightStickDeadbands);
    driverController.getStickProcessing(XboxController.Axis.kRightX).configure(stickConfig);
    driverController.getStickProcessing(XboxController.Axis.kRightY).configure(stickConfig);
    AxisProcChain.Config triggerConfig =
        new AxisProcChain.Config(kDriverTriggerSensitivity, kDriverTriggerDeadbands);
    driverController
        .getTriggerProcessing(XboxController.Axis.kLeftTrigger)
        .configure(triggerConfig);
    driverController
        .getTriggerProcessing(XboxController.Axis.kRightTrigger)
        .configure(triggerConfig);

    // Configure operator controller stick and trigger processing
    operatorController = new ProcessedXboxController(ControllerId.kOperator.port);
    stickConfig =
        new AxisProcChain.Config(kOperatorLeftStickSensitivity, kOperatorLeftStickDeadbands);
    driverController.getStickProcessing(XboxController.Axis.kLeftX).configure(stickConfig);
    driverController.getStickProcessing(XboxController.Axis.kLeftY).configure(stickConfig);
    stickConfig =
        new AxisProcChain.Config(kOperatorRightStickSensitivity, kOperatorRightStickDeadbands);
    driverController.getStickProcessing(XboxController.Axis.kRightX).configure(stickConfig);
    driverController.getStickProcessing(XboxController.Axis.kRightY).configure(stickConfig);
    triggerConfig =
        new AxisProcChain.Config(kOperatorTriggerSensitivity, kOperatorTriggerDeadbands);
    driverController
        .getTriggerProcessing(XboxController.Axis.kLeftTrigger)
        .configure(triggerConfig);
    driverController
        .getTriggerProcessing(XboxController.Axis.kRightTrigger)
        .configure(triggerConfig);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * @param botContainer Object providing access to robot subsystems
   */
  public void configureButtonBindings(RobotContainer botContainer) {

    // Map buttons on driver controller
    driverController.A.onTrue(
        new PlaceObject(
            botContainer.armSubsystem,
            botContainer.botStateSubsystem.storedGamePiece,
            Arm.Rank.Low,
            botContainer.s_Pneumatics,
            Arm.ArmExtenderPosition.OnFloor));
    driverController.B.onTrue(new InstantCommand(this::doNothing, this));
    driverController.X.onTrue(
        new PlaceObject(
            botContainer.armSubsystem,
            botContainer.botStateSubsystem.storedGamePiece,
            Arm.Rank.Medium,
            botContainer.s_Pneumatics,
            Arm.ArmExtenderPosition.MiddleRank));
    driverController.Y.onTrue(
        new PlaceObject(
            botContainer.armSubsystem,
            botContainer.botStateSubsystem.storedGamePiece,
            Arm.Rank.High,
            botContainer.s_Pneumatics,
            Arm.ArmExtenderPosition.TopRank));
    driverController.leftBumper.whileTrue(
        new ContinuousGetObject(
            Arm.GamePieceType.Cone,
            botContainer.ArmCamera,
            botContainer.swerveSubsystem,
            driverController,
            true,
            botContainer.openLoop,
            botContainer.armSubsystem,
            botContainer.botStateSubsystem,
            botContainer.s_Pneumatics));
    operatorController.rightBumper.whileTrue(
        new ContinuousGetObject(
            Arm.GamePieceType.Cube,
            botContainer.ArmCamera,
            botContainer.swerveSubsystem,
            driverController,
            true,
            botContainer.openLoop,
            botContainer.armSubsystem,
            botContainer.botStateSubsystem,
            botContainer.s_Pneumatics));
    driverController.leftStickPress.onTrue(new InstantCommand(this::doNothing, this));
    driverController.rightStickPress.onTrue(new InstantCommand(this::doNothing, this));
    driverController.back.onTrue(
      new InstantCommand(() -> botContainer.swerveSubsystem.zeroGyro()));
    driverController.leftTriggerAsButton.whileTrue(
        new SnapToGrid(
          botContainer.swerveSubsystem, 
          botContainer.joystickSubsystem, 
          true, 
          true));

    // Map buttons on operator controller
    operatorController.A.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.B.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.X.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.Y.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.leftBumper.whileTrue(new RotateIntake(botContainer.s_Pneumatics, true));
    operatorController.rightBumper.whileTrue(new RotateIntake(botContainer.s_Pneumatics, false));
    operatorController.leftStickPress.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.rightStickPress.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.back.onTrue(new InstantCommand(this::doNothing, this));
    operatorController.start.onTrue(new InstantCommand(this::doNothing, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Placeholder used for empty commands mapped to joystick */
  public void doNothing() {}
}
