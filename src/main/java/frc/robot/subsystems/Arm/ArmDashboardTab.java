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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm.Arm.ArmSubsystemDashboardInputs;
import frc.robot.subsystems.Arm.Arm.ArmSubsystemDashboardOutputs;

/** Dashboard tab for tuning the Arm subsystem */
public class ArmDashboardTab {
  /** The dashboard tab */
  private ShuffleboardTab m_tab;

  /** Measurements populated by the subsystem */
  private final ArmSubsystemDashboardOutputs m_measurements;

  /** NetworkTables entry used to reset the angle encoder */
  private GenericEntry m_angleZero;

  /** Create an instance of the tab */
  public ArmDashboardTab(ArmSubsystemDashboardOutputs measurements) {
    m_measurements = measurements;
  }

  /** Called to initialize the dashboard tab */
  public void initialize() {
    m_tab = Shuffleboard.getTab("Arm Subsystem");

    // Height of subpanels
    final int kSubPanelHeight = 6;

    // Width of subpanels
    final int kSubPanelWidth = 6;

    // Create Angle subpanel
    ShuffleboardLayout angleLayout = m_tab.getLayout("Angle Motor", BuiltInLayouts.kGrid);
    angleLayout.withSize(kSubPanelWidth, kSubPanelHeight).withPosition(0, 0);

    try {
      // Set up a widget to display the encoder count
      angleLayout
          .addDouble(
              "Encoder",
              () -> {
                return m_measurements.angleEncoderCount;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 0);

      // Set up a widget to display the angle in degrees
      angleLayout
          .addDouble(
              "Degrees",
              () -> {
                return m_measurements.angleDegrees;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 0);

      // Set up a widget for resetting the encoders
      m_angleZero =
          angleLayout
              .add("Zero Encoder", false)
              .withWidget(BuiltInWidgets.kBooleanBox)
              .withPosition(0, 2)
              .getEntry();

    } catch (IllegalArgumentException e) {
      DriverStation.reportError(
          "Failed to configure Arm Subsystem angle panel: " + e.getMessage(), false);
    }
  }

  /** Updates dashboard widgets with subsystem measurements */
  public void update(ArmSubsystemDashboardInputs inputs) {
    inputs.zeroAngleEncoder = m_angleZero.getBoolean(false);
  }
}
