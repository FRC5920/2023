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

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.dashboard.WidgetsWithChangeDetection.SliderWithChangeDetection;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import frc.robot.subsystems.Intake.IntakeSubsystem.IntakeSubsystemTelemetry;
import frc.robot.subsystems.Intake.IntakeSubsystem.MotorTelemetry;
import java.util.Map;
import java.util.function.Supplier;

/** Dashboard tab for tuning the Arm subsystem */
public class IntakeDashboardTab implements IDashboardTab {

  /** Width of a motor telemetry panel */
  private static final int kTelemetryPanelWidthCells = 6;

  /** Height of a motor telemetry panel */
  private static final int kTelemetryPanelHeightCells = 6;

  /** Shuffleboard tab to display */
  private ShuffleboardTab m_tab;

  /** The intake subsystem addressed by the dashboard */
  private final IntakeSubsystem m_intakeSubsystem;

  /** Measurements populated by the subsystem */
  private final IntakeSubsystemTelemetry m_telemetry;

  /** A command for testing intake speed */
  private TestCommand m_intakeTestCommand;

  /** A slider for testing intake speed */
  SliderWithChangeDetection m_intakeSpeedSlider;

  /** Create an instance of the tab */
  public IntakeDashboardTab(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    m_telemetry = new IntakeSubsystemTelemetry();
  }

  /** Called to initialize the dashboard tab */
  public void initDashboard(RobotContainer botContainer) {
    final String kTabTitle = "Intake";
    m_tab = Shuffleboard.getTab(kTabTitle);

    // Create telemetry panels for master and slave motors
    new MotorTelemetryPanel(
        m_tab,
        "Master motor",
        () -> m_telemetry.masterMotor,
        0,
        0 * kTelemetryPanelWidthCells,
        kTelemetryPanelWidthCells,
        kTelemetryPanelHeightCells);

    new MotorTelemetryPanel(
        m_tab,
        "Slave motor",
        () -> m_telemetry.slaveMotor,
        0,
        1 * kTelemetryPanelWidthCells + 2,
        kTelemetryPanelWidthCells,
        kTelemetryPanelHeightCells);

    m_intakeSpeedSlider =
        new SliderWithChangeDetection(
            m_tab, "Intake Speed (percent)", SpeedPreset.Acquire.motorSpeed, 0, 100, 1);
    m_intakeSpeedSlider.getWidget().withPosition(4, 10).withSize(5, 2);

    m_intakeTestCommand =
        new TestCommand(m_intakeSubsystem, () -> m_intakeSpeedSlider.getValue() / 100.0);
    m_tab.add(m_intakeTestCommand).withPosition(0, 10);
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void updateDashboard(RobotContainer botContainer) {
    // Update telemetry
    m_intakeSubsystem.getTelemetry(m_telemetry);
  }

  public static class MotorTelemetryPanel {
    /**
     * Creates a panel for displaying motor telemetry
     *
     * @param tab Shuffleboard tab the panel will be created in
     * @param panelTitle Title to display in the panel
     * @param telemetrySupplier Telemetry supplier used to drive panel widgets
     * @param panelRow Row position of the panel on the dashboard
     * @param panelCol Column position of the panel on the dashboard
     * @param widthCells Width of the panel in cells
     * @param heightCells Height of the panel in cells
     */
    public MotorTelemetryPanel(
        ShuffleboardTab tab,
        String panelTitle,
        Supplier<MotorTelemetry> telemetrySupplier,
        int panelRow,
        int panelColumn,
        int widthCells,
        int heightCells) {
      ShuffleboardLayout layout = tab.getLayout(panelTitle, BuiltInLayouts.kGrid);
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
                return telemetrySupplier.get().statorCurrentAmps;
              })
          .withWidget(BuiltInWidgets.kGraph)
          .withPosition(0, 0);

      // Set up a widget to display the applied motor voltage
      layout
          .addDouble(
              "Voltage",
              () -> {
                return telemetrySupplier.get().motorVolts;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 1);

      // Set up a widget to display the motor temperature
      layout
          .addDouble(
              "Temp",
              () -> {
                return telemetrySupplier.get().temperatureCelcius;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 2);

      // Set up a widget to display the motor RPM
      layout
          .addDouble(
              "Speed (percent)",
              () -> {
                return telemetrySupplier.get().speedPercent;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 3);
    }
  }
}
