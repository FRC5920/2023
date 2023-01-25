// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Dashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.lib.SwerveDrive.Constants;

public class DriveTab extends SubsystemBase {
  /** Creates a new DriveTab. */
  private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
  private GenericEntry maxSpeed =
      tab.add("Max Speed", 0.75)
      .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
         .getEntry();
         


  public DriveTab() {}

  @Override
  public void periodic() {
    if (RobotState.isDisabled()){
      BotStateSubsystem.MaxSpeed = Constants.Swerve.maxSpeed * maxSpeed.getDouble(0);
      BotStateSubsystem.MaxRotate = Constants.Swerve.maxAngularVelocity * maxSpeed.getDouble(0);
    }
  }
}
