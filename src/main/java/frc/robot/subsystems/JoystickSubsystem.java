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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.Joystick.JoystickSubsystemBase;
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
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

/** A subsystem providing/managing Xbox controllers for driving the robot manually */
public class JoystickSubsystem extends JoystickSubsystemBase {

  /** true to enable the operator controller */
  public static final boolean kOperatorControllerIsEnabled = false;

  /** A placeholder command used for button bindings */
  public static final InstantCommand kDoNothing = new InstantCommand();

  /** Creates an instance of the subsystem */
  public JoystickSubsystem() {
    super(true, kOperatorControllerIsEnabled);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * @param botContainer Object providing access to robot subsystems
   */
  @Override
  public void configureButtonBindings(RobotContainer botContainer) {
    ShooterPivotSubsystem shooterPivot = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intake = botContainer.intakeSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    // Create shoot commands that are active when left trigger is off
    CommandBase closeShotLow =
        new BotLog.DebugPrintCommand("<Shoot Low>")
            .andThen(new Shoot(ShooterPresets.PivotSideLow, shooterPivot, intake));
    CommandBase closeShotMid =
        new BotLog.DebugPrintCommand("<Shoot Mid>")
            .andThen(new Shoot(ShooterPresets.PivotSideMid, shooterPivot, intake));
    CommandBase closeShotHigh =
        new BotLog.DebugPrintCommand("<Shoot High>")
            .andThen(new Shoot(ShooterPresets.PivotSideHigh, shooterPivot, intake));

    // --------------------
    // Create shift-keyed shoot commands that are active when left trigger is pulled
    CommandBase hailMaryShotLow =
        new BotLog.DebugPrintCommand("<HailMary Low>")
            .andThen(new Shoot(ShooterPresets.PivotSideHailMaryLow, shooterPivot, intake));
    CommandBase hailMaryShotMid =
        new BotLog.DebugPrintCommand("<HailMary Mid>")
            .andThen(new Shoot(ShooterPresets.PivotSideHailMaryMid, shooterPivot, intake));
    CommandBase hailMaryShotHigh =
        new BotLog.DebugPrintCommand("<HailMary High>")
            .andThen(new Shoot(ShooterPresets.PivotSideHailMaryHigh, shooterPivot, intake));

    // Map buttons on driver controller
    ProcessedXboxController driverController = getDriverController();

    // Map buttons and button combos for shots
    setupButtonCombo(
        driverController.A, driverController.leftTriggerAsButton, closeShotLow, hailMaryShotLow);
    setupButtonCombo(
        driverController.B, driverController.leftTriggerAsButton, closeShotMid, hailMaryShotMid);
    setupButtonCombo(
        driverController.Y, driverController.leftTriggerAsButton, closeShotHigh, hailMaryShotHigh);

    CommandBase acquireAndParkCommand =
        new BotLog.DebugPrintCommand("<Acquire and park>")
            .andThen(Acquire.acquireAndPark(shooterPivot, intake));
    CommandBase emergencyParkCommand =
        new BotLog.DebugPrintCommand("<Acquire and park>")
            .andThen(new ShooterPivotSubsystem.EmergencyPark(botContainer.shooterPivotSubsystem));

    // Map button and button combos for intake
    setupButtonCombo(
        driverController.X,
        driverController.leftTriggerAsButton,
        acquireAndParkCommand,
        emergencyParkCommand);

    // Map right bumper
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

    // Map left bumper
    driverController.leftBumper.whileTrue(
        new DriveWithZTargeting(
            GameTarget.AprilTag2D, botContainer.ArmCamera, swerveSubsystem, this, true, true));

    // Map stick press buttons
    driverController.leftStickPress.onTrue(kDoNothing);
    driverController.rightStickPress.onTrue(kDoNothing);

    // Map BACK button (small button on the left in the middle of the controller)
    driverController.back.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));

    // Map BACK button (small button on the right in the middle of the controller)
    driverController.start.whileTrue(new Balance(swerveSubsystem));

    // Map right trigger to snap-to-grid
    driverController.rightTriggerAsButton.whileTrue(
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
            .finallyDo((interrupted) -> BotLog.SimInfo("Snap-to-grid OFF")));

    // Map buttons on operator controller
    if (kOperatorControllerIsEnabled) {
      ProcessedXboxController operatorController = getOperatorController();

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

      operatorController.leftStickPress.onTrue(kDoNothing);
      operatorController.rightStickPress.onTrue(kDoNothing);
      operatorController.back.onTrue(kDoNothing);
      operatorController.start.onTrue(kDoNothing);
    }
  }
}
