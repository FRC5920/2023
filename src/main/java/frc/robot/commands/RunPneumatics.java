// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics.Pneumatics;


public class RunPneumatics extends CommandBase {
  Pneumatics PneumaticsSubsystem;
  boolean activatePistons;

  /** Creates a new RunPneumatics. */
  public RunPneumatics(Pneumatics s_Pneumatics, boolean PistonsForward) {
    addRequirements(s_Pneumatics);
    this.PneumaticsSubsystem = s_Pneumatics;
    this.activatePistons = PistonsForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (activatePistons){
      PneumaticsSubsystem.goingForward();
    }else{
      PneumaticsSubsystem.goingBackward();
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
