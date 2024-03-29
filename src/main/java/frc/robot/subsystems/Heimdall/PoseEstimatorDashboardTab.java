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
package frc.robot.subsystems.Heimdall;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Dashboard.IDashboardTab;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.Map;

/** A class supplying a Shuffleboard tab for configuring drive train parameters */
public class PoseEstimatorDashboardTab implements IDashboardTab {
  /** Title used for a dashboard tab that displays the field */
  static final String kTabTitle = "PoseEstimator";

  /** Width (in cells) of the field display */
  static final int kFieldWidthCells = 21;

  /** Height (in cells) of the field display */
  static final int kFieldHeightCells = 12;

  /** Width (in cells) of the pose value display */
  static final int kPoseWidthCells = 5;

  /** Height (in cells) of the pose value display */
  static final int kPoseHeightCells = 2;

  /** Handle to the pose estimator subsystem */
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;

  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Creates an instance of the tab */
  PoseEstimatorDashboardTab(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_PoseEstimatorSubsystem = poseEstimatorSubsystem;
    m_field2d = new Field2d();
  }

  /**
   * Create and initialize dashboard widgets
   *
   * @param botContainer Container that holds robot subsystems
   */
  @Override
  public void initDashboard(RobotContainer botContainer) {
    PoseEstimatorSubsystem poseSubsystem = botContainer.poseEstimatorSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    m_tab = Shuffleboard.getTab(kTabTitle);

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(kFieldWidthCells, kFieldHeightCells)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));

    // Add the pose estimator's pose
    m_tab
        .addString("Pose Estimator (m)", () -> formatPose2d(poseSubsystem.getCurrentPose(), false))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 0, kFieldHeightCells);
    m_tab
        .addString("Pose Estimator (in)", () -> formatPose2d(poseSubsystem.getCurrentPose(), true))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 0, kFieldHeightCells + kPoseHeightCells);

    // Add the pose value
    m_tab
        .addString("Swerve Pose (m)", () -> formatPose2d(swerveSubsystem.getPose(), false))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 1, kFieldHeightCells);
    m_tab
        .addString("Swerve Pose (in)", () -> formatPose2d(swerveSubsystem.getPose(), true))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 1, kFieldHeightCells + kPoseHeightCells);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    FieldObject2d estimatorPoseObject = m_field2d.getRobotObject();
    estimatorPoseObject.setPose(m_PoseEstimatorSubsystem.getCurrentPose());

    FieldObject2d swervePoseObject = m_field2d.getObject("Odometry");
    swervePoseObject.setPose(botContainer.swerveSubsystem.getPose());
  }

  private static String formatPose2d(Pose2d pose, boolean asInches) {
    if (asInches) {
      return String.format(
          "(%.2f in, %.2f in) %.2f degrees",
          Units.metersToInches(pose.getX()),
          Units.metersToInches(pose.getY()),
          pose.getRotation().getDegrees());
    } else {
      return String.format(
          "(%.2f m, %.2f m) %.2f degrees",
          pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
  }
}
