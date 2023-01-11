// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Dashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;;

public class DriveTab extends SubsystemBase {
  /** Creates a new DriveTab. */
  public DriveTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    SendableChooser<Double> m_driveSpeed = new SendableChooser<>();
    m_driveSpeed.addOption("Full Speed", 1.0);
    m_driveSpeed.addOption("90 %", 0.9);
    m_driveSpeed.addOption("80 %", 0.8);
    m_driveSpeed.addOption("70 %", 0.7);
    m_driveSpeed.addOption("60 %", 0.6);
    m_driveSpeed.addOption("50 %", 0.5);
    m_driveSpeed.addOption("40 %", 0.4);
    m_driveSpeed.addOption("30 %", 0.3);
    m_driveSpeed.setDefaultOption("20 %", .2);
    m_driveSpeed.addOption("10 %", 0.1);
    tab.add("Speed Limit", m_driveSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
