// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Joystick.ProcessedXboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;

public class SnapToGrid extends CommandBase {
  /** Creates a new SnapToGrid. */
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  private boolean foundSnapPoint = false;
  double yAxis;

  private Swerve s_Swerve;
  private ProcessedXboxController controller;


  public SnapToGrid(
    Swerve s_Swerve,
    JoystickSubsystem joystickSubsystem,
    boolean fieldRelative,
    boolean openLoop
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.controller = joystickSubsystem.driverController;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < FieldConstants.Grids.nodeRowCount; i++) {
    if (((FieldConstants.Grids.lowTranslations[i].getY() - (FieldConstants.Grids.nodeSeparationY * 0.5)) < s_Swerve.getPose().getY()) && 
    (s_Swerve.getPose().getY() <= (FieldConstants.Grids.lowTranslations[i].getY() + (FieldConstants.Grids.nodeSeparationY * 0.5)))) {
      yAxis = FieldConstants.Grids.lowTranslations[i].getY();
    } else {
      yAxis = -controller.getLeftY();
    }
    double xAxis = -controller.getLeftX();
    double rAxis = -controller.getRightX();

    translation = new Translation2d(yAxis, xAxis).times(BotStateSubsystem.MaxSpeed);
    rotation = rAxis * BotStateSubsystem.MaxRotate;
    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

