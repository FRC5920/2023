////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5290 - Vikotics    **                       |
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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import java.util.Map;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class DriveSubsystemDashboardTab implements IDashboardTab {

  /** A reference to the swerve subsystem */
  private Swerve m_swerveSubsystem;

  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;

  /** Dashboard widget for adjusting the swerve drive speed coefficient */
  private GenericEntry m_maxSpeed;

  /** 2d view of the field */
  private Field2d m_field2d;

  // /** The current X position (meters) */
  // NetworkTableEntry m_poseX;
  // /** The current Y position (meters) */
  // NetworkTableEntry m_poseY;
  // /** The current rotation (degrees) */
  // NetworkTableEntry m_poseRotation;

  // /////////////////////////////////////////////////////////////
  // // Telemetry for left and right sides of the Drive Train
  // /////////////////////////////////////////////////////////////
  // /** NetworkTable entry used to display distance traveled in meters */
  // NetworkTableEntry m_distanceMeters[];
  // /** NetworkTable entry used to display velocity in meters/sec */
  // NetworkTableEntry m_velocityMetersPerSec[];
  // /** NetworkTable entry used to display velocity error */
  // NetworkTableEntry m_velocityError[];

  /**
   * Creates an instance of the tab
   *
   * @param driveTrainSubsystem Drive Train subsystem to operate on
   */
  DriveSubsystemDashboardTab() {}

  /** Create and initialize dashboard widgets */
  @Override
  public void initialize(RobotContainer botContainer) {
    m_swerveSubsystem = botContainer.swerveSubsystem;

    m_tab = Shuffleboard.getTab("Drive Train");

    m_maxSpeed =
        m_tab
            .add("Max Speed", 0.75)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();

    m_field2d = new Field2d();
    m_tab.add(m_field2d);

    /*

    ShuffleboardLayout poseLayout = m_tab.getLayout("Pose", BuiltInLayouts.kGrid)
                .withSize(4, 1)
                .withPosition(0, 0);

    m_poseX = poseLayout.add("X (m)", 0.0)
      .withSize(2, 2)
      .withPosition(0, 0)
      .getEntry();

    m_poseY = poseLayout.add("Y (m)", 0.0)
      .withSize(2, 2)
      .withPosition(1, 0)
      .getEntry();

    m_poseRotation = poseLayout.add("Rot (deg)", 0.0)
      .withSize(2, 2)
      .withPosition(2, 0)
      .getEntry();

    String sideName[] = {"Left", "Right"};
    m_distanceMeters = new NetworkTableEntry[2];
    m_velocityMetersPerSec = new NetworkTableEntry[2];
    m_velocityError = new NetworkTableEntry[2];
    for (int side = kLeft; side <= kRight; ++side) {
      ShuffleboardLayout telemetryLayout = m_tab.getLayout(sideName[side] + " Telemetry", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(side * 2, 1)
        .withProperties(Map.of("Label position", "LEFT"));
      m_distanceMeters[side] = telemetryLayout.add("Distance", 0.0)
        .withSize(2,1)
        .withPosition(0, 0)
        .getEntry();
      m_velocityMetersPerSec[side] = telemetryLayout.add("Velocity", 0.0)
        .withSize(2,1)
        .withPosition(0, 1)
        .getEntry();
      m_velocityError[side] = telemetryLayout.add("Velocity Error", 0.0)
        .withSize(2,1)
        .withPosition(0, 2)
        .getEntry();
    }

    // Set up a gain tuner layout for each side of the drive train
    ShuffleboardLayout leftTuner = m_tab.getLayout("Left Drivebase", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"));
    try {
      leftTuner.add("Left PID Control", m_driveTrainSubsystem.getSendableGains()[0]);
      leftTuner.addNumber("Left PID Error", () -> { return m_driveTrainSubsystem.getVelocitySensorError()[0]; });
    } catch (Exception e) {}

    ShuffleboardLayout rightTuner = m_tab.getLayout("Right Drivebase", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"));
    try {
      rightTuner.add("Right PID Control", m_driveTrainSubsystem.getSendableGains()[1]);
      rightTuner.addNumber("Right PID Error", () -> { return m_driveTrainSubsystem.getVelocitySensorError()[1]; });
    } catch (Exception e) {}
    */
  }

  /** Service dashboard tab widgets */
  @Override
  public void update() {
    /*
    double distanceSI[] = m_driveTrainSubsystem.getSensorDistanceMeters();
    //double velocitySI[] = m_driveTrainSubsystem.getEncoderVelocitySI();
    double velocityError[] = m_driveTrainSubsystem.getVelocitySensorError();
    DifferentialDriveWheelSpeeds speeds = m_driveTrainSubsystem.getWheelSpeeds();
    for (int side = kLeft; side <= kRight; ++side) {
      m_distanceMeters[side].setDouble(distanceSI[side]);
      m_velocityMetersPerSec[side].setDouble(side == kLeft ? speeds.leftMetersPerSecond : speeds.rightMetersPerSecond);
      m_velocityError[side].setDouble(velocityError[side]);
    }

    Pose2d pose = m_driveTrainSubsystem.getPose();
    Rotation2d rot = pose.getRotation();
    m_poseX.setDouble(pose.getX());
    m_poseY.setDouble(pose.getY());
    m_poseRotation.setDouble(rot.getDegrees());
    */
    m_field2d.setRobotPose(m_swerveSubsystem.getPose());

    if (RobotState.isDisabled()) {
      if (m_maxSpeed != null) {
        BotStateSubsystem.MaxSpeed =
            Constants.SwerveDrivebaseConstants.maxSpeed * m_maxSpeed.getDouble(0);
        BotStateSubsystem.MaxRotate =
            Constants.SwerveDrivebaseConstants.maxAngularVelocity * m_maxSpeed.getDouble(0);
      }
    }
  }
}
