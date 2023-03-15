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
package frc.robot.subsystems.ShooterPivot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.dashboard.WidgetsWithChangeDetection.PIDTunerPanel;
import frc.lib.dashboard.WidgetsWithChangeDetection.SliderWithChangeDetection;
import frc.lib.dashboard.WidgetsWithChangeDetection.ToggleButtonWithChangeDetection;
import frc.lib.utility.PIDGains;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem.MotorTelemetry;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem.ShooterPivotTelemetry;
import java.util.Map;
import java.util.function.Supplier;

/** Dashboard tab for tuning the Arm subsystem */
public class ShooterPivotDashboardTab implements IDashboardTab {

  /** Width of a motor telemetry panel */
  private static final int kTelemetryPanelWidthCells = 6;

  /** Height of a motor telemetry panel */
  private static final int kTelemetryPanelHeightCells = 6;

  /** Shuffleboard tab to display */
  private ShuffleboardTab m_tab;

  /** Telemetry display for the master motor */
  private MotorTelemetryPanel m_masterMotorPanel;

  /** Telemetry display for the slave motor */
  private MotorTelemetryPanel m_slaveMotorPanel;

  /** The intake subsystem addressed by the dashboard */
  private final ShooterPivotSubsystem m_shooterSubsystem;

  /** Measurements populated by the subsystem */
  private final ShooterPivotTelemetry m_telemetry;

  /** Slider used to set test motor speed for intake */
  private SliderWithChangeDetection m_positionSlider;

  /** Toggle button used to enable/disable motor at speed */
  private ToggleButtonWithChangeDetection m_motorEnableToggle;

  /** PID tuner panel */
  private PIDTunerPanel m_pidTuner;

  private PIDGains m_pidGains = ShooterPivotSubsystem.kDefaultPIDGains;

  /** Create an instance of the tab */
  public ShooterPivotDashboardTab(ShooterPivotSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
    m_telemetry = new ShooterPivotTelemetry();
    m_masterMotorPanel = null;
    m_slaveMotorPanel = null;
  }

  /** Called to initialize the dashboard tab */
  public void initDashboard(RobotContainer botContainer) {
    final String kTabTitle = "ShooterPivot";
    m_tab = Shuffleboard.getTab(kTabTitle);

    m_masterMotorPanel =
        new MotorTelemetryPanel(
            m_tab,
            "Master motor",
            () -> m_telemetry.masterMotor,
            0,
            0 * kTelemetryPanelWidthCells,
            kTelemetryPanelWidthCells,
            kTelemetryPanelHeightCells);

    m_slaveMotorPanel =
        new MotorTelemetryPanel(
            m_tab,
            "Slave motor",
            () -> m_telemetry.slaveMotor,
            0,
            1 * kTelemetryPanelWidthCells + 2,
            kTelemetryPanelWidthCells,
            kTelemetryPanelHeightCells);

    m_pidTuner =
        new PIDTunerPanel(m_tab, "Position PID", kTelemetryPanelHeightCells + 4, 0, m_pidGains);

    m_positionSlider = new SliderWithChangeDetection(m_tab, "Test Position (Deg)", 100, -35, 135);
    m_positionSlider
        .getWidget()
        .withPosition(3, kTelemetryPanelHeightCells + 4)
        .withSize(kTelemetryPanelWidthCells + 4, 3);

    m_motorEnableToggle = new ToggleButtonWithChangeDetection(m_tab, "Apply", false);
    m_motorEnableToggle
        .getWidget()
        .withPosition(3 + (kTelemetryPanelWidthCells + 4), kTelemetryPanelHeightCells + 4)
        .withSize(3, 3);
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void updateDashboard(RobotContainer botContainer) {

    // Update PID gains if they have changed
    if (m_pidTuner.hasChanged()) {
      m_shooterSubsystem.setPIDGains(m_pidTuner.getGains());
    }

    // Test the motor speed if any widgets have changed
    if (m_motorEnableToggle.hasChanged() || m_positionSlider.hasChanged()) {
      if (m_motorEnableToggle.getValue()) {
        m_shooterSubsystem.setAngleDegrees(m_positionSlider.getValue());
      }
    }

    // Update telemetry
    m_shooterSubsystem.getTelemetry(m_telemetry);
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
              Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 5));

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

      // Set up a widget to display the sensor position
      layout
          .addDouble(
              "Ticks",
              () -> {
                return telemetrySupplier.get().sensorPositionTicks;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 3);

      // Set up a widget to display the position in degrees
      layout
          .addDouble(
              "Degrees",
              () -> {
                return telemetrySupplier.get().positionDegrees;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 4);
    }
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
