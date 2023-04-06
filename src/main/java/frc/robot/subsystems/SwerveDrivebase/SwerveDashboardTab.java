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
package frc.robot.subsystems.SwerveDrivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.SwerveDrive.SwerveModuleIO.SwerveModuleIOTelemetry;
import frc.lib.dashboard.SimGyroVisualizer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import frc.robot.subsystems.SwerveDrivebase.Swerve.ModuleId;
import java.util.Map;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class SwerveDashboardTab implements IDashboardTab {

  /** Width (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kSwerveModuleTelemetryWidth = 6;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  static final int kSwerveModuleLayoutHeight = 14;

  /** Title used for a dashboard tab that displays swerve drive info */
  static final String kSwerveTabTitle = "SwerveDrive";

  /** Title used for a dashboard tab that displays the field */
  static final String kFieldTabTitle = "Field";

  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;

  /** Dashboard widget for adjusting the swerve drive speed coefficient */
  private GenericEntry m_maxSpeed;

  /** Swerve module telemetry displays */
  private ModuleTelemetryLayout m_moduleTelemetry[];

  private SimGyroVisualizer m_simGyroVisualizer = null;

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
  public void initDashboard(RobotContainer botContainer) {
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    m_tab = Shuffleboard.getTab(kSwerveTabTitle);

    // Add telemetry display of swerve modules
    m_moduleTelemetry =
        new ModuleTelemetryLayout[] {
          new ModuleTelemetryLayout(
              m_tab, ModuleId.kFrontLeft, kSwerveModuleTelemetryWidth, kSwerveModuleLayoutHeight),
          new ModuleTelemetryLayout(
              m_tab, ModuleId.kFrontRight, kSwerveModuleTelemetryWidth, kSwerveModuleLayoutHeight),
          new ModuleTelemetryLayout(
              m_tab, ModuleId.kRearLeft, kSwerveModuleTelemetryWidth, kSwerveModuleLayoutHeight),
          new ModuleTelemetryLayout(
              m_tab, ModuleId.kRearRight, kSwerveModuleTelemetryWidth, kSwerveModuleLayoutHeight),
        };

    int col = 0;
    for (ModuleId moduleId : ModuleId.values()) {
      m_moduleTelemetry[moduleId.value].getLayout().withPosition(col, 0);
      col += kSwerveModuleTelemetryWidth;
    }

    // --------------------------------------
    // Add chassis speeds layout
    ShuffleboardLayout cspeedLayout =
        m_tab
            .getLayout("Chassis Speeds", BuiltInLayouts.kGrid)
            .withProperties(
                Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3))
            .withSize(6, 4)
            .withPosition(0, kSwerveModuleLayoutHeight);
    cspeedLayout
        .addDouble(
            "xVel",
            () -> {
              return swerveSubsystem.getChassisSpeeds().vxMetersPerSecond;
            })
        .withSize(kSwerveModuleTelemetryWidth, 1)
        .withPosition(0, 0);
    cspeedLayout
        .addDouble(
            "yVel",
            () -> {
              return swerveSubsystem.getChassisSpeeds().vyMetersPerSecond;
            })
        .withSize(kSwerveModuleTelemetryWidth, 1)
        .withPosition(0, 1);
    cspeedLayout
        .addDouble(
            "Omega",
            () -> {
              return Units.radiansToDegrees(
                  swerveSubsystem.getChassisSpeeds().omegaRadiansPerSecond);
            })
        .withSize(kSwerveModuleTelemetryWidth, 1)
        .withPosition(0, 2);

    // --------------------------------------
    // Add max speed slider
    m_maxSpeed =
        m_tab
            .add("Max Speed", 1)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(
                Map.of(
                    "Min", 0, "Max", 1, "Block increment", 0.05)) // specify widget properties here
            .withSize(kSwerveModuleTelemetryWidth * 2, 4)
            .withPosition(kSwerveModuleTelemetryWidth, kSwerveModuleLayoutHeight)
            .getEntry();

    if (RobotBase.isSimulation()) {
      m_simGyroVisualizer = new SimGyroVisualizer();
      m_tab.add("Degrees", m_simGyroVisualizer).withPosition(0, 15).withSize(3, 3);
    }
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    Swerve swerveSubsystem = botContainer.swerveSubsystem;
    m_field2d.setRobotPose(swerveSubsystem.getPose());

    // Update swerve module telemetry
    for (ModuleId moduleId : ModuleId.values()) {
      int idx = moduleId.value;
      m_moduleTelemetry[idx].update(swerveSubsystem.getIOTelemetry(moduleId));
    }

    if (RobotState.isDisabled()) {
      if (m_maxSpeed != null) {
        RobotContainer.MaxSpeed =
            Constants.SwerveDrivebaseConstants.maxSpeed * m_maxSpeed.getDouble(0);
        RobotContainer.MaxRotate =
            Constants.SwerveDrivebaseConstants.maxAngularVelocity * m_maxSpeed.getDouble(0);
      }
    }

    if (RobotBase.isSimulation()) {
      m_simGyroVisualizer.update(botContainer.swerveSubsystem.getYaw(), new Rotation2d());
    }
  }

  private class ModuleTelemetryLayout {
    private final ShuffleboardLayout m_layout;

    private final GenericEntry driveSpeed;
    private final GenericEntry driveDistance;
    private final GenericEntry driveVolts;
    private final GenericEntry driveCurrent;
    private final GenericEntry driveTemp;

    private final GenericEntry angleVel;
    private final GenericEntry angleVolts;
    private final GenericEntry angleCurrent;
    private final GenericEntry angleTemp;

    private final SimGyroVisualizer m_swerveVisualizer;

    /**
     * Creates a Shuffleboard layout for displaying telemetry of a swerve module
     *
     * @param tab Shuffleboard tab to create the layout in
     * @param moduleId ID of the swerve module being represented
     */
    public ModuleTelemetryLayout(
        ShuffleboardTab tab, ModuleId moduleId, int sizeColumns, int sizeRows) {
      m_swerveVisualizer = new SimGyroVisualizer();

      // Create a grid layout to add subgrouped widgets to
      m_layout = tab.getLayout(moduleId.toString(), BuiltInLayouts.kGrid);
      m_layout
          .withProperties(
              Map.of("Label position", "TOP", "Number of columns", 1, "Number of rows", 2))
          .withSize(sizeColumns, sizeRows);

      BuiltInWidgets widgetType = BuiltInWidgets.kTextView;
      int telemetryFieldHeight = 1;

      // --------------------------------------
      // Add angle layout
      ShuffleboardLayout angleLayout =
          m_layout
              .getLayout("Angle", BuiltInLayouts.kGrid)
              .withProperties(
                  Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 5))
              .withPosition(0, 0);

      angleLayout
          .add("Degrees", m_swerveVisualizer)
          .withPosition(0, 0)
          .withSize(sizeColumns, telemetryFieldHeight * 3);

      angleVel =
          angleLayout
              .add("Rate", 0)
              .withWidget(widgetType)
              .withPosition(0, 1)
              .withSize(sizeColumns, telemetryFieldHeight)
              .getEntry();
      angleVolts =
          angleLayout
              .add("Volts", 0)
              .withWidget(widgetType)
              .withPosition(0, 2)
              .withSize(sizeColumns, telemetryFieldHeight)
              .getEntry();
      angleCurrent =
          angleLayout
              .add("Current", 0)
              .withWidget(widgetType)
              .withPosition(0, 3)
              .withSize(sizeColumns, telemetryFieldHeight)
              .getEntry();
      angleTemp =
          angleLayout
              .add("Temp", 0)
              .withWidget(widgetType)
              .withPosition(0, 4)
              .withSize(sizeColumns, telemetryFieldHeight)
              .getEntry();

      // --------------------------------------
      // Add drive layout
      ShuffleboardLayout driveLayout =
          m_layout
              .getLayout("Drive", BuiltInLayouts.kGrid)
              .withProperties(
                  Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 5))
              .withPosition(0, 1);

      driveSpeed =
          driveLayout
              .add("Speed", 0)
              .withWidget(widgetType)
              .withPosition(0, 0)
              .withSize(sizeColumns + 1, telemetryFieldHeight)
              .getEntry();

      driveDistance =
          driveLayout
              .add("Distance", 0)
              .withWidget(widgetType)
              .withPosition(0, 1)
              .withSize(sizeColumns + 1, telemetryFieldHeight)
              .getEntry();

      driveVolts =
          driveLayout
              .add("Volts", 0)
              .withWidget(widgetType)
              .withPosition(0, 2)
              .withSize(sizeColumns + 1, telemetryFieldHeight)
              .getEntry();
      driveCurrent =
          driveLayout
              .add("Current", 0)
              .withWidget(widgetType)
              .withPosition(0, 3)
              .withSize(sizeColumns + 1, telemetryFieldHeight)
              .getEntry();
      driveTemp =
          driveLayout
              .add("Temp", 0)
              .withWidget(widgetType)
              .withPosition(0, 4)
              .withSize(sizeColumns + 1, telemetryFieldHeight)
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
      driveCurrent.setDouble(telemetry.driveCurrentAmps);
      driveTemp.setDouble(telemetry.driveTempCelcius);
      m_swerveVisualizer.update(
          Rotation2d.fromRadians(telemetry.anglePositionRad),
          Rotation2d.fromRadians(telemetry.angleVelocityRadPerSec));
      angleVel.setDouble(Units.radiansToDegrees(telemetry.angleVelocityRadPerSec));
      angleVolts.setDouble(telemetry.angleAppliedVolts);
      angleCurrent.setDouble(telemetry.angleCurrentAmps);
      angleTemp.setDouble(telemetry.angleTempCelcius);
    }

    ShuffleboardLayout getLayout() {
      return m_layout;
    }
  }
}
