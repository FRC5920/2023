// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class ChangeColor extends CommandBase {
  /** Creates a new ChangeColor. */
  LEDs m_lights;
  Color8Bit myColor;

  public ChangeColor(LEDs lights, Color8Bit desiredColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
    m_lights = lights;
    myColor = desiredColor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lights.updateColor(myColor);
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
