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
package frc.robot.subsystems.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import frc.robot.subsystems.Intake.IntakeSubsystem.IntakeSubsystemTelemetry;
import frc.robot.subsystems.Intake.IntakeSubsystem.MotorTelemetry;
import java.util.Map;

/** Dashboard tab for tuning the Arm subsystem */
public class IntakeDashboardTab implements IDashboardTab {
  // Height of subpanels
  final int kPanelHeight = 4;

  // Width of subpanels
  final int kPanelWidth = 10;

  /** The dashboard tab */
  private ShuffleboardTab m_tab;

  /** The intake subsystem addressed by the dashboard */
  private final IntakeSubsystem m_intakeSubsystem;

  /** Measurements populated by the subsystem */
  private final IntakeSubsystemTelemetry m_telemetry;

  /** NetworkTables entry used to control motor speed for intake */
  private GenericEntry m_frontMotorSpeedNTE;

  /** NetworkTables entry used to control motor speed for placement */
  private GenericEntry m_rearMotorSpeedNTE;

  /** Create an instance of the tab */
  public IntakeDashboardTab(IntakeSubsystem subsystem, IntakeSubsystemTelemetry telemetry) {
    m_intakeSubsystem = subsystem;
    m_telemetry = telemetry;
  }

  /** Called to initialize the dashboard tab */
  public void initDashboard(RobotContainer botContainer) {
    final String kTabTitle = "Intake Subsystem";
    m_tab = Shuffleboard.getTab(kTabTitle);

    createTelemetryPanel(
        m_telemetry.frontMotor, "Front Roller", 0, kPanelWidth * 0, kPanelWidth, kPanelHeight);
    createTelemetryPanel(
        m_telemetry.frontMotor, "Rear Roller", 0, kPanelWidth * 1, kPanelWidth, kPanelHeight);
    m_frontMotorSpeedNTE =
        createMotorSpeedSlider("Front Roller Speed (RPM)", 9, kPanelWidth * 0, kPanelWidth, 4);
    m_rearMotorSpeedNTE =
        createMotorSpeedSlider("Rear Roller Speed (RPM)", 9, kPanelWidth * 1, kPanelWidth, 4);
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void updateDashboard(RobotContainer botContainer) {
    // Set motor speeds
    double frontMotorSpeed = m_frontMotorSpeedNTE.getDouble(0.0);
    double rearMotorSpeed = m_rearMotorSpeedNTE.getDouble(0.0);
    m_intakeSubsystem.setFrontRollerRPM(frontMotorSpeed);
    m_intakeSubsystem.DEBUG_runRearRoller(rearMotorSpeed);
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
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 4));

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

    // Set up a widget to display the motor RPM
    layout
        .addDouble(
            "RPM",
            () -> {
              return telemetry.motorRPM;
            })
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 3);
  }

  /**
   * Creates a panel for intake speed
   *
   * @param panelRow Row position of the panel on the dashboard
   * @param panelCol Column position of the panel on the dashboard
   * @param widthCells Width of the panel in cells
   * @param heightCells Height of the panel in cells
   */
  private GenericEntry createMotorSpeedSlider(
      String panelTitle, int row, int column, int widthCells, int heightCells) {

    Map<String, Object> speedSliderMap =
        Map.of("min", 0.00, "max", 3000.0, "Block increment", 10.0);
    // Set up a slider to control motor speed during intake
    GenericEntry ntEntry =
        m_tab
            .add(panelTitle, 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(speedSliderMap) // slider min/max/increment
            .withPosition(column, row)
            .withSize(widthCells, heightCells)
            .getEntry();

    return ntEntry;
  }
}
