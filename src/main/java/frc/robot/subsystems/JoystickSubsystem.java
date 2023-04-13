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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Joystick.AxisProcChain;
import frc.lib.Joystick.ProcessedXboxController;
import frc.lib.utility.BotLogger.BotLog;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.commands.Balance;
import frc.robot.commands.Shooter.Acquire;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterPresets;
import frc.robot.commands.SnapToGrid;
import frc.robot.commands.zTarget.DriveWithZTargeting;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

/** A subsystem providing/managing Xbox controllers for driving the robot manually */
public class JoystickSubsystem extends SubsystemBase {

  public static final boolean kDriverControllerIsEnabled = true;
  public static final boolean kOperatorControllerIsEnabled = false;

  public enum ControllerId {
    kDriver(0),
    kOperator(1);

    public final int port;

    ControllerId(int portNum) {
      this.port = portNum;
    }
  }

  // ---------- Joystick Processing Constants -----------------
  public static final double kDriverLeftStickSensitivity = 0.5;
  public static final double kDriverLeftStickDeadbands[] = {0.1, 0.95};
  public static final double kDriverRightStickSensitivity = 0.5;
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

    if (kDriverControllerIsEnabled) {
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
    }

    if (kOperatorControllerIsEnabled) {
      // Configure operator controller stick and trigger processing
      operatorController = new ProcessedXboxController(ControllerId.kOperator.port);
      AxisProcChain.Config stickConfig =
          new AxisProcChain.Config(kOperatorLeftStickSensitivity, kOperatorLeftStickDeadbands);
      driverController.getStickProcessing(XboxController.Axis.kLeftX).configure(stickConfig);
      driverController.getStickProcessing(XboxController.Axis.kLeftY).configure(stickConfig);
      stickConfig =
          new AxisProcChain.Config(kOperatorRightStickSensitivity, kOperatorRightStickDeadbands);
      driverController.getStickProcessing(XboxController.Axis.kRightX).configure(stickConfig);
      driverController.getStickProcessing(XboxController.Axis.kRightY).configure(stickConfig);
      AxisProcChain.Config triggerConfig =
          new AxisProcChain.Config(kOperatorTriggerSensitivity, kOperatorTriggerDeadbands);
      driverController
          .getTriggerProcessing(XboxController.Axis.kLeftTrigger)
          .configure(triggerConfig);
      driverController
          .getTriggerProcessing(XboxController.Axis.kRightTrigger)
          .configure(triggerConfig);
    }
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
    ShooterPivotSubsystem shooterPivot = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intake = botContainer.intakeSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    // Create shoot commands that are active when left trigger is off
    CommandBase closeShotLow =
        new Shoot(ShooterPresets.PivotSideLow, shooterPivot, intake)
            .unless(() -> driverController.leftTriggerAsButton.getAsBoolean());
    CommandBase closeShotMid =
        new Shoot(ShooterPresets.PivotSideMid, shooterPivot, intake)
            .unless(() -> driverController.leftTriggerAsButton.getAsBoolean());
    CommandBase closeShotHigh =
        new Shoot(ShooterPresets.PivotSideHigh, shooterPivot, intake)
            .unless(() -> driverController.leftTriggerAsButton.getAsBoolean());

    // --------------------
    // Create shift-keyed shoot commands that are active when left trigger is pulled
    CommandBase hailMaryShotLow =
        new Shoot(ShooterPresets.PivotSideHailMaryLow, shooterPivot, intake)
            .unless(() -> !driverController.leftTriggerAsButton.getAsBoolean());
    CommandBase hailMaryShotMid =
        new Shoot(ShooterPresets.PivotSideHailMaryMid, shooterPivot, intake)
            .unless(() -> !driverController.leftTriggerAsButton.getAsBoolean());
    CommandBase hailMaryShotHigh =
        new Shoot(ShooterPresets.PivotSideHailMaryHigh, shooterPivot, intake)
            .unless(() -> !driverController.leftTriggerAsButton.getAsBoolean());

    CommandBase snap2GridCommand =
        new BotLog.SimInfoPrintCommand("Snap-to-grid ON")
            .andThen(
                new SnapToGrid(
                    swerveSubsystem,
                    this,
                    true,
                    true,
                    RobotContainer.MaxSpeed,
                    RobotContainer.MaxRotate,
                    botContainer.autoDashboardTab.getField2d()))
            .finallyDo((interrupted) -> BotLog.SimInfo("Snap-to-grid OFF"));

    CommandBase acquireAndParkCommand =
        Acquire.acquireAndPark(shooterPivot, intake)
            .unless(() -> !driverController.X.getAsBoolean());

    if (kDriverControllerIsEnabled) {
      // Map buttons on driver controller

      // Map shift-keyed buttons for shots
      driverController.A.onTrue(
          Commands.either(
              hailMaryShotLow, closeShotLow, driverController.leftTriggerAsButton::getAsBoolean));
      driverController.B.onTrue(
          Commands.either(
              hailMaryShotMid, closeShotMid, driverController.leftTriggerAsButton::getAsBoolean));
      driverController.Y.onTrue(
          Commands.either(
              hailMaryShotHigh, closeShotHigh, driverController.leftTriggerAsButton::getAsBoolean));

      // Map X button:
      //   X (alone) = acquireAndPark
      //   X+leftTrigger = emergency park
      driverController.X.whileTrue(
          Commands.either(
              acquireAndParkCommand,
              new ShooterPivotSubsystem.EmergencyPark(botContainer.shooterPivotSubsystem),
              driverController.X::getAsBoolean));

      driverController.rightBumper.whileTrue(
          DriveWithZTargeting.zTargetDriveWithIntake(
              GameTarget.Cube,
              botContainer.ArmCamera,
              swerveSubsystem,
              this,
              shooterPivot,
              intake,
              true,
              true));

      driverController.leftBumper.whileTrue(
          new DriveWithZTargeting(
              GameTarget.AprilTag2D, botContainer.ArmCamera, swerveSubsystem, this, true, true));

      driverController.leftStickPress.onTrue(new InstantCommand(this::doNothing, this));
      driverController.rightStickPress.onTrue(new InstantCommand(this::doNothing, this));
      driverController.back.onTrue(
          new InstantCommand(() -> swerveSubsystem.zeroGyro())); // left little
      driverController.start.whileTrue(new Balance(swerveSubsystem)); // right little

      driverController.rightTriggerAsButton.whileTrue(snap2GridCommand);
    }

    if (kOperatorControllerIsEnabled) {
      // Map buttons on operator controller
      operatorController.A.onTrue(new Shoot(ShooterPresets.RSLSideLow, shooterPivot, intake));
      operatorController.B.onTrue(new Shoot(ShooterPresets.RSLSideMid, shooterPivot, intake));
      operatorController.Y.onTrue(new Shoot(ShooterPresets.RSLSideHigh, shooterPivot, intake));
      operatorController.X.onTrue(
          new Shoot(ShooterPresets.PivotSideUpAgainstGridLow, shooterPivot, intake));

      operatorController.leftBumper.whileTrue(
          Acquire.acquireAndPark(botContainer.shooterPivotSubsystem, botContainer.intakeSubsystem));
      operatorController.rightBumper.whileTrue(new InstantCommand());

      operatorController.leftTriggerAsButton.whileTrue(
          Acquire.acquireAndPark(shooterPivot, intake));

      operatorController.leftStickPress.onTrue(new InstantCommand(this::doNothing, this));
      operatorController.rightStickPress.onTrue(new InstantCommand(this::doNothing, this));
      operatorController.back.onTrue(
          new InstantCommand(
              () -> botContainer.shooterPivotSubsystem.setAnglePreset(PivotPresets.Park)));
      operatorController.start.onTrue(new InstantCommand());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /** Placeholder used for empty commands mapped to joystick */
  public void doNothing() {}
}
