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
import frc.robot.subsystems.Intake.IntakeSubsystem.IntakeSubsystemTelemetry;
import frc.robot.subsystems.Intake.IntakeSubsystem.MotorTelemetry;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import frc.robot.RobotContainer;

import java.util.Map;

/** Dashboard tab for tuning the Arm subsystem */
public class IntakeDashboardTab implements IDashboardTab {
  // Height of subpanels
  final int kPanelHeight = 3;

  // Width of subpanels
  final int kPanelWidth = 6;

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
  public IntakeDashboardTab(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    m_telemetry = new IntakeSubsystemTelemetry();
  }

  /** Called to initialize the dashboard tab */
  public void initDashboard(RobotContainer botContainer) {
    final String kTabTitle = "Intake Subsystem";
    m_tab = Shuffleboard.getTab(kTabTitle);

    createTelemetryPanel(m_telemetry.frontMotor, 0, 0, kPanelWidth, 6);
    //createIntakeSpeedPanel();

    Shuffleboard.selectTab(kTabTitle);
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void updateDashboard(RobotContainer botContainer) {
    // Set motor speeds
    double frontMotorSpeed = m_frontMotorSpeedNTE.getDouble(0.0);
    double rearMotorSpeed = m_rearMotorSpeedNTE.getDouble(0.0);
    m_intakeSubsystem.runIntakeFront(frontMotorSpeed);
    // TODO: run rear motor
  }

  private void createTelemetryPanel(MotorTelemetry telemetry, 
    int panelRow, int panelColumn, int widthCells, int heightCells) {
    ShuffleboardLayout layout = m_tab.getLayout("Intake Motor Current", BuiltInLayouts.kGrid);
    layout
        .withSize(kPanelWidth * 3, 8)
        .withPosition(panelColumn, panelRow)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3));

    // Set up a widget to display the instantaneous motor current value in amps
    layout
        .addDouble(
            "Voltage",
            () -> {
              return telemetry.motorVolts;
            })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 0);

    layout
        .addDouble(
            "Current",
            () -> {
              return telemetry.statorCurrentAmps;
            })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 1);

    layout
        .addDouble(
            "Temp",
            () -> {
              return telemetry.temperatureCelcius;
            })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 1);
  }

  private void createIntakeSpeedPanel(int panelRow, int panelColumn, int widthCells, int heightCells) {
    ShuffleboardLayout layout = m_tab.getLayout("Intake Motor Speed", BuiltInLayouts.kGrid);
    layout
        .withSize(widthCells, heightCells)
        .withPosition(panelColumn, panelRow)
        .withProperties(
            Map.of("Label position", "LEFT", "Number of columns", 2, "Number of rows", 1));

    Map<String, Object> speedSliderMap = Map.of("min", 0.00, "max", 1000.0, "Block increment", 1.0);
    // Set up a slider to control motor speed during intake
    m_frontMotorSpeedNTE =
        layout
            .add("Front Speed (RPM)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(speedSliderMap) // slider min/max/increment
            .withPosition(0, 0)
            .getEntry();

    // Set up a slider to control motor speed during placement
    m_rearMotorSpeedNTE =
        layout
            .add("Rear Speed (RPM)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(speedSliderMap) // slider min/max/increment
            .withPosition(1, 0)
            .getEntry();
  }


}
