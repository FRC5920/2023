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
package frc.robot.subsystems.Dashboard;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.SwerveDrive.SwerveModuleIO.SwerveModuleIOTelemetry;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.SwerveDrivebase.Swerve.ModuleId;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import java.util.Map;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class SwerveDashboardTab implements IDashboardTab {

  /** Width (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kTelemetryWidth = 6;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kTelemetryHeight = 9;

  /** Title used for a dashboard tab that displays swerve drive info */
  static final String kSwerveTabTitle = "SwerveDrive";

  /** Title used for a dashboard tab that displays the field */
  static final String kFieldTabTitle = "Field";

  /** A reference to the swerve subsystem */
  private Swerve m_swerveSubsystem;

  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;

  /** Dashboard widget for adjusting the swerve drive speed coefficient */
  private GenericEntry m_maxSpeed;

  /** Swerve module telemetry displays */
  private ModuleTelemetryLayout m_moduleTelemetry[];

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Creates an instance of the tab */
  SwerveDashboardTab() {
    m_field2d = new Field2d();
  }

  /**
   * Create and initialize dashboard widgets
   *
   * @param botContainer Container that holds robot subsystems
   */
  @Override
  public void initialize(RobotContainer botContainer) {
    m_swerveSubsystem = botContainer.swerveSubsystem;

    m_tab = Shuffleboard.getTab(kSwerveTabTitle);

    // Add telemetry display of swerve modules
    m_moduleTelemetry =
        new ModuleTelemetryLayout[] {
          new ModuleTelemetryLayout(m_tab, ModuleId.kFrontLeft, kTelemetryWidth, kTelemetryHeight),
          new ModuleTelemetryLayout(m_tab, ModuleId.kFrontRight, kTelemetryWidth, kTelemetryHeight),
          new ModuleTelemetryLayout(m_tab, ModuleId.kRearLeft, kTelemetryWidth, kTelemetryHeight),
          new ModuleTelemetryLayout(m_tab, ModuleId.kRearRight, kTelemetryWidth, kTelemetryHeight),
        };

    int col = 0;
    for (ModuleId moduleId : ModuleId.values()) {
      m_moduleTelemetry[moduleId.value].getLayout().withPosition(col, 0);
      col += kTelemetryWidth;
    }

    m_maxSpeed =
        m_tab
            .add("Max Speed", 0.75)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(
                Map.of(
                    "Min", 0, "Max", 1, "Block increment", 0.05)) // specify widget properties here
            .withSize(4, 4)
            .withPosition(0, kTelemetryHeight)
            .getEntry();

    // Display the field in a tab
    m_tab.add(m_field2d).withSize(14, 9).withPosition(4, kTelemetryHeight);

    Shuffleboard.selectTab(kSwerveTabTitle);
  }

  /** Service dashboard tab widgets */
  @Override
  public void update() {
    m_field2d.setRobotPose(m_swerveSubsystem.getPose());

    // Update swerve module telemetry
    for (ModuleId moduleId : ModuleId.values()) {
      int idx = moduleId.value;
      m_moduleTelemetry[idx].update(m_swerveSubsystem.getIOTelemetry(moduleId));
    }

    if (RobotState.isDisabled()) {
      if (m_maxSpeed != null) {
        BotStateSubsystem.MaxSpeed =
            Constants.SwerveDrivebaseConstants.maxSpeed * m_maxSpeed.getDouble(0);
        BotStateSubsystem.MaxRotate =
            Constants.SwerveDrivebaseConstants.maxAngularVelocity * m_maxSpeed.getDouble(0);
      }
    }
  }

  private class ModuleTelemetryLayout {
    private final ShuffleboardLayout m_layout;

    private final GenericEntry driveSpeed;
    private final GenericEntry driveDistance;
    private final GenericEntry driveVolts;
    private final GenericEntry driveCurrent;
    private final GenericEntry driveTemp;

    private final GenericEntry absDegrees;
    private final GenericEntry rawDegrees;
    private final GenericEntry angleVel;
    private final GenericEntry angleVolts;
    private final GenericEntry angleCurrent;
    private final GenericEntry angleTemp;

    /**
     * Creates a Shuffleboard layout for displaying telemetry of a swerve module
     *
     * @param tab Shuffleboard tab to create the layout in
     * @param moduleId ID of the swerve module being represented
     */
    public ModuleTelemetryLayout(
        ShuffleboardTab tab, ModuleId moduleId, int numColumns, int numRows) {
      // Create a vertical list layout to add subgrouped widgets to
      m_layout = tab.getLayout(moduleId.toString(), BuiltInLayouts.kGrid);
      m_layout
          .withProperties(
              Map.of("Label position", "LEFT", "Number of columns", "1", "Number of rows", "11"))
          .withSize(numColumns, numRows);

      BuiltInWidgets widgetType = BuiltInWidgets.kTextView;
      int telemetryFieldHeight = 1;

      driveDistance =
          m_layout
              .add("Drive Distance", 0)
              .withWidget(widgetType)
              .withPosition(0, 0)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      driveSpeed =
          m_layout
              .add("Drive Speed", 0)
              .withWidget(widgetType)
              .withPosition(0, 1)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      absDegrees =
          m_layout
              .add("Angle (abs)", 0)
              .withWidget(widgetType)
              .withPosition(0, 2)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      rawDegrees =
          m_layout
              .add("Angle (raw)", 0)
              .withWidget(widgetType)
              .withPosition(0, 3)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      angleVel =
          m_layout
              .add("Angular Vel", 0)
              .withWidget(widgetType)
              .withPosition(0, 4)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();

      driveVolts =
          m_layout
              .add("Drive Volts", 0)
              .withWidget(widgetType)
              .withPosition(0, 5)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      driveCurrent =
          m_layout
              .add("Drive Current", 0)
              .withWidget(widgetType)
              .withPosition(0, 6)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      driveTemp =
          m_layout
              .add("Drive Temp", 0)
              .withWidget(widgetType)
              .withPosition(0, 7)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();

      angleVolts =
          m_layout
              .add("Volts", 0)
              .withWidget(widgetType)
              .withPosition(0, 8)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      angleCurrent =
          m_layout
              .add("Current", 0)
              .withWidget(widgetType)
              .withPosition(0, 9)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      angleTemp =
          m_layout
              .add("Temp", 0)
              .withWidget(widgetType)
              .withPosition(0, 10)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
    }

    /**
     * Update dashboard values from telemetry
     *
     * @param telemetry Telemetry values to apply to dashboard widgets
     */
    void update(SwerveModuleIOTelemetry telemetry) {
      driveSpeed.setDouble(telemetry.driveSpeedMetersPerSecond);
      driveDistance.setDouble(telemetry.driveDistanceMeters);
      driveVolts.setDouble(telemetry.driveAppliedVolts);
      driveCurrent.setDouble(
          telemetry.driveCurrentAmps.length >= 1 ? telemetry.driveCurrentAmps[0] : 0.0);
      driveTemp.setDouble(
          telemetry.driveTempCelcius.length >= 1 ? telemetry.driveTempCelcius[0] : 0.0);
      absDegrees.setDouble(Units.radiansToDegrees(telemetry.angleAbsolutePositionRad));
      rawDegrees.setDouble(Units.radiansToDegrees(telemetry.anglePositionRad));
      angleVel.setDouble(Units.radiansToDegrees(telemetry.angleVelocityRadPerSec));
      angleVolts.setDouble(telemetry.angleAppliedVolts);
      angleCurrent.setDouble(
          telemetry.angleCurrentAmps.length >= 1 ? telemetry.angleCurrentAmps[0] : 0.0);
      angleTemp.setDouble(
          telemetry.angleTempCelcius.length >= 1 ? telemetry.angleTempCelcius[0] : 0.0);
    }

    ShuffleboardLayout getLayout() {
      return m_layout;
    }
  }
}
