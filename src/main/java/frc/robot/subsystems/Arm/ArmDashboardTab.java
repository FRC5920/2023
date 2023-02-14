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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm.Arm.ArmSubsystemDashboardInputs;
import frc.robot.subsystems.Arm.Arm.ArmSubsystemDashboardOutputs;
import java.util.Map;

/** Dashboard tab for tuning the Arm subsystem */
public class ArmDashboardTab {
  // Height of subpanels
  final int kPanelHeight = 3;

  // Width of subpanels
  final int kPanelWidth = 6;

  /** The dashboard tab */
  private ShuffleboardTab m_tab;

  /** Measurements populated by the subsystem */
  private final ArmSubsystemDashboardOutputs m_measurements;

  /** NetworkTables entry used to control motor speed for intake */
  private GenericEntry m_nteIntakeMotorSpeed;

  /** NetworkTables entry used to control motor speed for placement */
  private GenericEntry m_ntePlacementMotorSpeed;

  /** Create an instance of the tab */
  public ArmDashboardTab(ArmSubsystemDashboardOutputs measurements) {
    m_measurements = measurements;
  }

  /** Called to initialize the dashboard tab */
  public void initialize() {
    final String kTabTitle = "Arm Subsystem";
    m_tab = Shuffleboard.getTab(kTabTitle);

    createAnglePanel();
    createExtenderPanel();
    createWristPanel();
    createIntakeCurrentPanel();
    createIntakeSpeedPanel();

    Shuffleboard.selectTab(kTabTitle);
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void update(ArmSubsystemDashboardInputs inputs) {
    inputs.intakeCargoMotorSpeed = m_nteIntakeMotorSpeed.getDouble(0.1);
    inputs.placeCargoMotorSpeed = m_ntePlacementMotorSpeed.getDouble(0.1);
  }

  private void createAnglePanel() {
    ShuffleboardLayout layout = m_tab.getLayout("Angle Motor", BuiltInLayouts.kGrid);
    layout
        .withSize(kPanelWidth, kPanelHeight)
        .withPosition(0 * kPanelWidth, 0)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

    // Set up a widget to display the encoder count
    layout
        .addDouble(
            "Encoder",
            () -> {
              return m_measurements.angleEncoderCount;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0);

    // Set up a widget to display the angle in degrees
    layout
        .addDouble(
            "Degrees",
            () -> {
              return m_measurements.angleDegrees;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1);
  }

  private void createExtenderPanel() {
    ShuffleboardLayout layout = m_tab.getLayout("Extender Motor", BuiltInLayouts.kGrid);
    layout
        .withSize(kPanelWidth, kPanelHeight)
        .withPosition(1 * kPanelWidth, 0)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

    // Set up a widget to display the encoder count
    layout
        .addDouble(
            "Encoder",
            () -> {
              return m_measurements.extenderEncoderCount;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0);

    // Set up a widget to display the angle in degrees
    layout
        .addDouble(
            "Position (meters)",
            () -> {
              return m_measurements.extenderPositionMeters;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1);
  }

  private void createWristPanel() {
    ShuffleboardLayout layout = m_tab.getLayout("Wrist Pneumatics", BuiltInLayouts.kGrid);
    layout
        .withSize(kPanelWidth, kPanelHeight)
        .withPosition(2 * kPanelWidth, 0)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 1));

    // Add a widget to show the present wrist orientation
    layout
        .addString(
            "Orientation",
            () -> {
              return m_measurements.wristPosition.toString();
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0);
  }

  private void createIntakeSpeedPanel() {
    ShuffleboardLayout layout = m_tab.getLayout("Intake Motor Speed", BuiltInLayouts.kGrid);
    layout
        .withSize(kPanelWidth * 3, 3)
        .withPosition(0, 1 * kPanelHeight)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 2, "Number of rows", 1));

    // Set up a slider to control motor speed during intake
    m_nteIntakeMotorSpeed =
        layout
            .add("Intake Speed (%)", 0.5)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.01, "max", 1.0)) // specify widget properties here
            .withPosition(0, 0)
            .getEntry();

    // Set up a slider to control motor speed during placement
    m_ntePlacementMotorSpeed =
        layout
            .add("Placement Speed (%)", 0.2)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.01, "max", 1.0)) // specify widget properties here
            .withPosition(1, 0)
            .getEntry();
  }

  private void createIntakeCurrentPanel() {
    ShuffleboardLayout layout = m_tab.getLayout("Intake Motor Current", BuiltInLayouts.kGrid);
    layout
        .withSize(kPanelWidth * 3, 8)
        .withPosition(0 * kPanelWidth, 2 * kPanelHeight)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

    // Set up a widget to display the instantaneous motor current value in amps
    layout
        .addDouble(
            "Upper Intake Current",
            () -> {
              return m_measurements.upperIntakeCurrentAmps;
            })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 0);

    layout
        .addDouble(
            "Lower Intake Current",
            () -> {
              return m_measurements.lowerIntakeCurrentAmps;
            })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 1);
  }
}
