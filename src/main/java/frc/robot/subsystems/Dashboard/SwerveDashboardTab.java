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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
  static final int kTelemetryHeight = 11;

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
    m_tab.add(m_field2d).withSize(15, kTelemetryHeight).withPosition(kTelemetryWidth * 4, 0);

    m_tab
        .addDouble(
            "CSpeed xVel",
            () -> {
              return m_swerveSubsystem.getChassisSpeeds().vxMetersPerSecond;
            })
        .withSize(kTelemetryWidth, 3)
        .withPosition(4, 12);
    m_tab
        .addDouble(
            "CSpeed yVel",
            () -> {
              return m_swerveSubsystem.getChassisSpeeds().vyMetersPerSecond;
            })
        .withSize(kTelemetryWidth, 3)
        .withPosition(4 + kTelemetryWidth, 12);
    m_tab
        .addDouble(
            "CSpeed omega",
            () -> {
              return Units.radiansToDegrees(
                  m_swerveSubsystem.getChassisSpeeds().omegaRadiansPerSecond);
            })
        .withSize(kTelemetryWidth, 3)
        .withPosition(4 + (2 * kTelemetryWidth), 12);

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

    private final GenericEntry angleVel;
    private final GenericEntry angleVolts;
    private final GenericEntry angleCurrent;
    private final GenericEntry angleTemp;

    private final SwerveModuleVisualizer m_swerveVisualizer;

    /**
     * Creates a Shuffleboard layout for displaying telemetry of a swerve module
     *
     * @param tab Shuffleboard tab to create the layout in
     * @param moduleId ID of the swerve module being represented
     */
    public ModuleTelemetryLayout(
        ShuffleboardTab tab, ModuleId moduleId, int numColumns, int numRows) {
      m_swerveVisualizer = new SwerveModuleVisualizer();

      // Create a vertical list layout to add subgrouped widgets to
      m_layout = tab.getLayout(moduleId.toString(), BuiltInLayouts.kGrid);
      m_layout
          .withProperties(
              Map.of("Label position", "LEFT", "Number of columns", "1", "Number of rows", "10"))
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
      m_layout
          .add("Angle Deg", m_swerveVisualizer)
          .withPosition(0, 2)
          .withSize(numColumns, telemetryFieldHeight * 3);
      angleVel =
          m_layout
              .add("Angle Rate", 0)
              .withWidget(widgetType)
              .withPosition(0, 3)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();

      driveVolts =
          m_layout
              .add("Drive Volts", 0)
              .withWidget(widgetType)
              .withPosition(0, 4)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      driveCurrent =
          m_layout
              .add("Drive Current", 0)
              .withWidget(widgetType)
              .withPosition(0, 5)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      driveTemp =
          m_layout
              .add("Drive Temp", 0)
              .withWidget(widgetType)
              .withPosition(0, 6)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();

      angleVolts =
          m_layout
              .add("Volts", 0)
              .withWidget(widgetType)
              .withPosition(0, 7)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      angleCurrent =
          m_layout
              .add("Current", 0)
              .withWidget(widgetType)
              .withPosition(0, 8)
              .withSize(numColumns, telemetryFieldHeight)
              .getEntry();
      angleTemp =
          m_layout
              .add("Temp", 0)
              .withWidget(widgetType)
              .withPosition(0, 9)
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
      driveCurrent.setDouble(telemetry.driveCurrentAmps);
      driveTemp.setDouble(telemetry.driveTempCelcius);
      m_swerveVisualizer.update(
          Units.radiansToDegrees(telemetry.angleAbsolutePositionRad),
          Units.radiansToDegrees(telemetry.angleVelocityRadPerSec));
      angleVel.setDouble(Units.radiansToDegrees(telemetry.angleVelocityRadPerSec));
      angleVolts.setDouble(telemetry.angleAppliedVolts);
      angleCurrent.setDouble(telemetry.angleCurrentAmps);
      angleTemp.setDouble(telemetry.angleTempCelcius);
    }

    ShuffleboardLayout getLayout() {
      return m_layout;
    }
  }

  private class SwerveModuleVisualizer implements Gyro, Sendable {
    double m_angle;
    double m_rate;

    /**
     * Creates an instance of the object
     *
     * @param angleSupplier supplies the angle of the module in degrees
     * @param rateSupplier supplies the angle rate of change in degrees per second
     */
    public SwerveModuleVisualizer() {}

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Gyro");
      builder.addDoubleProperty("Value", this::getAngle, null);
    }

    /** Update the angle and rate */
    public void update(double angleDegrees, double angleRateDegreesPerSec) {
      m_angle = angleDegrees;
      m_rate = angleRateDegreesPerSec;
    }

    /** Gyro.getAngle returns the angle in degrees */
    @Override
    public double getAngle() {
      return m_angle;
    }

    /** Gyro.getRate returns the rate of rotation in degrees per second */
    @Override
    public double getRate() {
      return m_rate;
    }

    /** Gyro.calibrate() does nothing in this implementation */
    @Override
    public void calibrate() {}

    /** Gyro.reset() does nothing in this implementation */
    @Override
    public void reset() {}

    /** AutoCloseable.close does nothing in this implementation */
    @Override
    public void close() {}
  }
}
