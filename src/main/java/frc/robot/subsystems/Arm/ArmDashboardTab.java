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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm.ArmSubsystemTelemetry;
import frc.robot.subsystems.Arm.Arm.MotorTelemetry;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import java.util.Map;

/** Dashboard tab for tuning the Arm subsystem */
public class ArmDashboardTab implements IDashboardTab {
  // Height of subpanels
  final int kPanelHeight = 4;

  // Width of subpanels
  final int kPanelWidth = 10;

  /** The dashboard tab */
  private ShuffleboardTab m_tab;

  /** Measurements populated by the subsystem */
  private final ArmSubsystemTelemetry m_telemetry;

  /** NetworkTables entry used to control angle motor position */
  private GenericEntry m_angleMotorPositionNTE;

  /** NetworkTables entry used to control extender motor position */
  private GenericEntry m_extenderMotorPositionNTE;

  /** Create an instance of the tab */
  public ArmDashboardTab(ArmSubsystemTelemetry telemetry) {
    m_telemetry = telemetry;
  }

  /** Called to initialize the dashboard tab */
  public void initDashboard(RobotContainer botContainer) {
    final String kTabTitle = "Arm Subsystem";
    m_tab = Shuffleboard.getTab(kTabTitle);

    createAnglePanel(kPanelHeight * 0, kPanelWidth * 0, kPanelWidth, kPanelHeight);
    createExtenderPanel(kPanelHeight * 0, kPanelWidth * 1, kPanelWidth, kPanelHeight);
    createTelemetryPanel(m_telemetry.angleTelemetry, "Angle Motor Telemetry", 4, 0, kPanelWidth, 4);
    createTelemetryPanel(
        m_telemetry.extenderTelemetry,
        "Extender Motor Telemetry",
        4,
        kPanelWidth * 1,
        kPanelWidth,
        4);
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void updateDashboard(RobotContainer botContainer) {
    // botContainer.armSubsystem.DEBUG_setAnglePosition(m_angleMotorPositionNTE.getDouble(0.0));
    // botContainer.armSubsystem.DEBUG_setExtenderPosition(m_extenderMotorPositionNTE.getDouble(0.0));
  }

  /**
   * Builds a panel with angle motor widgets on the dashboard tab
   *
   * @param panelRow Row position of the panel on the dashboard
   * @param panelCol Column position of the panel on the dashboard
   * @param widthCells Width of the panel in cells
   * @param heightCells Height of the panel in cells
   */
  private void createAnglePanel(int panelRow, int panelColumn, int widthCells, int heightCells) {
    ShuffleboardLayout layout = m_tab.getLayout("Angle Motor", BuiltInLayouts.kGrid);
    layout
        .withSize(widthCells, heightCells)
        .withPosition(panelColumn, panelRow)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

    // Set up a widget to display the encoder count
    layout
        .addDouble(
            "Encoder",
            () -> {
              return m_telemetry.angleEncoderCount;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0);

    // Set up a slider to control motor position
    m_angleMotorPositionNTE =
        layout
            .add("Position", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.00, "max", 10000.0, "Block increment", 10.0))
            .withPosition(0, 1)
            .getEntry();
  }

  /**
   * Builds a panel with extender motor widgets on the dashboard tab
   *
   * @param panelRow Row position of the panel on the dashboard
   * @param panelCol Column position of the panel on the dashboard
   * @param widthCells Width of the panel in cells
   * @param heightCells Height of the panel in cells
   */
  private void createExtenderPanel(int panelRow, int panelColumn, int widthCells, int heightCells) {
    ShuffleboardLayout layout = m_tab.getLayout("Extender Motor", BuiltInLayouts.kGrid);
    layout
        .withSize(widthCells, heightCells)
        .withPosition(panelColumn, panelRow)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

    // Set up a widget to display the encoder count
    layout
        .addDouble(
            "Encoder",
            () -> {
              return m_telemetry.extenderEncoderCount;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0);

    // Set up a slider to control motor position
    m_extenderMotorPositionNTE =
        layout
            .add("Position", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.00, "max", 10000.0, "Block increment", 10.0))
            .withPosition(0, 1)
            .getEntry();
  }

  /**
   * Creates a panel for displaying motor telemetry
   *
   * @param telemetry Telemetry to connect to widgets
   * @param panelTitle Title to display in the panel
   * @param panelRow Row position of the panel on the dashboard
   * @param panelCol Column position of the panel on the dashboard
   * @param widthCells Width of the panel in cells
   * @param heightCells Height of the panel in cells
   */
  private void createTelemetryPanel(
      MotorTelemetry telemetry,
      String panelTitle,
      int panelRow,
      int panelColumn,
      int widthCells,
      int heightCells) {
    ShuffleboardLayout layout = m_tab.getLayout(panelTitle, BuiltInLayouts.kGrid);
    layout
        .withSize(widthCells, heightCells)
        .withPosition(panelColumn, panelRow)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3));

    // Set up a widget to display motor current value in amps
    layout
        .addDouble(
            "Current",
            () -> {
              return telemetry.statorCurrentAmps;
            })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 0);

    // Set up a widget to display the applied motor voltage
    layout
        .addDouble(
            "Voltage",
            () -> {
              return telemetry.motorVolts;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1);

    // Set up a widget to display the motor temperature
    layout
        .addDouble(
            "Temp",
            () -> {
              return telemetry.temperatureCelcius;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2);
  }
}
