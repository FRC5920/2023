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
import frc.lib.dashboard.WidgetsWithChangeDetection.PIDTunerPanel;
import frc.lib.dashboard.WidgetsWithChangeDetection.SliderWithChangeDetection;
import frc.lib.utility.PIDGains;
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

  /** Telemetry display for the master motor */
  private MotorTelemetryPanel m_masterMotorPanel;

  /** Telemetry display for the slave motor */
  private MotorTelemetryPanel m_slaveMotorPanel;

  /** The intake subsystem addressed by the dashboard */
  private final IntakeSubsystem m_intakeSubsystem;

  /** Measurements populated by the subsystem */
  private final IntakeSubsystemTelemetry m_telemetry;

  /** PID tuner panel */
  private PIDTunerPanel m_pidTuner;

  private PIDGains m_pidGains = IntakeSubsystem.kDefaultPIDGains;

  // Commands for testing
  private TestCommand m_intakeTestCommand;

  SliderWithChangeDetection m_intakeSpeedSlider;

  /** Create an instance of the tab */
  public IntakeDashboardTab(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    m_telemetry = new IntakeSubsystemTelemetry();
    m_masterMotorPanel = null;
    m_slaveMotorPanel = null;
  }

  /** Called to initialize the dashboard tab */
  public void initDashboard(RobotContainer botContainer) {
    final String kTabTitle = "Intake";
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

    m_pidTuner = new PIDTunerPanel(m_tab, "Velocity PID", 0, 17, m_pidGains);

    m_intakeSpeedSlider =
        new SliderWithChangeDetection(
            m_tab,
            "Intake Speed (percent)",
            IntakeSubsystem.IntakePreset.CubeIntake.motorSpeed,
            0,
            100,
            1);
    m_intakeSpeedSlider.getWidget().withPosition(4, 10).withSize(5, 2);

    m_intakeTestCommand =
        new TestCommand(m_intakeSubsystem, () -> m_intakeSpeedSlider.getValue() / 100.0);
    m_tab.add(m_intakeTestCommand).withPosition(0, 10);

    // botContainer.joystickSubsystem.operatorController.leftBumper.whileTrue(
    //     new TestCommand(m_intakeSubsystem, () -> m_intakeSpeedSlider.getValue() / 100.0));
  }

  /** Updates dashboard widgets with subsystem measurements and obtains dashboard input values */
  public void updateDashboard(RobotContainer botContainer) {

    // Update PID gains if they have changed
    if (m_pidTuner.hasChanged()) {
      System.out.println("<IntakeDashboardTab::updateDashboard> Update intake PID Gains");
      m_intakeSubsystem.setPIDGains(m_pidTuner.getGains());
    }

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
              "RPM",
              () -> {
                return telemetrySupplier.get().motorRPM;
              })
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 3);
    }
  }
}
