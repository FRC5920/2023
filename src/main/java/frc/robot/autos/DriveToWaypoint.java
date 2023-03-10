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
package frc.robot.autos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utility.PIDGains;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

public class DriveToWaypoint extends CommandBase {
  /** Default tolerance to control position to in meters */
  public static final double kDefaultPositionToleranceMeters = 0.05;
  /** Default tolerance to control position to in meters */
  public static final double kDefaultRotationToleranceRadians = Units.degreesToRadians(5.0);

  /** Default gains applied to controlling position */
  public static final PIDGains kDefaultPositionGains = new PIDGains(8, 0, 0.2);
  /** Default gains applied to controlling position */
  public static final PIDGains kDefaultRotationGains = new PIDGains(10, 0, 0.2);

  /** Default max velocity used for position constraints */
  public static final double kDefaultMaxVelocity = 3.5;
  /** Default max acceleration used for position constraints */
  public static final double kDefaultMaxAcceleration = 11;
  /** Default control constraints used for the position controller */
  public static final TrapezoidProfile.Constraints kDefaultPositionConstraints =
      new TrapezoidProfile.Constraints(kDefaultMaxVelocity, kDefaultMaxAcceleration);

  /** Default max velocity used for rotation constraints */
  public static final double kDefaultMaxRotVelocity = Units.degreesToRadians(360.0);
  /** Default max acceleration used for rotation constraints */
  public static final double kDefaultMaxRotAcceleration = Units.degreesToRadians(720.0);
  /** Default control constraints used for the rotation controller */
  public static final TrapezoidProfile.Constraints kDefaultRotationConstraints =
      new TrapezoidProfile.Constraints(kDefaultMaxRotVelocity, kDefaultMaxRotAcceleration);

  private final Swerve m_swerve;
  private final Pose2d m_pose;
  private final double m_positionTolerance;
  private final double m_thetaTolerance;

  private boolean running = false;
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;

  /**
   * Drives to the specified pose under full software control
   *
   * @param swerveSubsystem Swerve drive subsystem used to drive
   * @param waypoint The pose to drive to
   * @param positionToleranceMeters Position tolerance to control to (meters)
   * @param rotationToleranceRad Rotation tolerance to control to (radians)
   */
  public DriveToWaypoint(
      Swerve swerveSubsystem,
      Pose2d waypoint,
      double positionToleranceMeters,
      double rotationToleranceRad) {
    this(
        swerveSubsystem,
        waypoint,
        positionToleranceMeters,
        rotationToleranceRad,
        kDefaultPositionGains,
        kDefaultRotationGains,
        kDefaultPositionConstraints,
        kDefaultRotationConstraints);
  }

  /**
   * Drives to the specified pose under full software control
   *
   * @param swerveSubsystem Swerve drive subsystem used to drive
   * @param pose The pose to drive to
   * @param positionToleranceMeters Position tolerance to control to (meters)
   * @param rotationToleranceRad Rotation tolerance to control to (radians)
   * @param positionPIDGains PID gains to apply to controlling position
   * @param rotationPIDGains PID gains to apply to controlling rotation
   * @param positionConstraints Constraints applied when controlling position
   * @param rotationConstraints Constraints applied when controlling rotation
   */
  public DriveToWaypoint(
      Swerve swerveSubsystem,
      Pose2d pose,
      double positionToleranceMeters,
      double rotationToleranceRad,
      PIDGains positionPIDGains,
      PIDGains rotationPIDGains,
      TrapezoidProfile.Constraints positionConstraints,
      TrapezoidProfile.Constraints rotationConstraints) {
    m_swerve = swerveSubsystem;
    m_pose = pose;
    m_positionTolerance = positionToleranceMeters;
    m_thetaTolerance = rotationToleranceRad;

    driveController =
        new ProfiledPIDController(
            positionPIDGains.kP,
            positionPIDGains.kI,
            positionPIDGains.kD,
            positionConstraints,
            Constants.robotPeriodSec);
    driveController.setConstraints(positionConstraints);
    driveController.setTolerance(m_positionTolerance);

    thetaController =
        new ProfiledPIDController(
            rotationPIDGains.kP,
            rotationPIDGains.kI,
            rotationPIDGains.kD,
            rotationConstraints,
            Constants.robotPeriodSec);

    thetaController.setConstraints(rotationConstraints);
    thetaController.setTolerance(m_thetaTolerance);

    addRequirements(m_swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = m_swerve.getPose();
    driveController.reset(currentPose.getTranslation().getDistance(m_pose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    running = true;
    driveController.setTolerance(m_positionTolerance);
    thetaController.setTolerance(m_thetaTolerance);

    // Get current and target pose
    var currentPose = m_swerve.getPose();
    var targetPose = m_pose;

    // Command speeds
    double driveVelocityScalar =
        driveController.calculate(
            currentPose.getTranslation().getDistance(m_pose.getTranslation()), 0.0);
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (driveController.atGoal()) driveVelocityScalar = 0.0;
    if (thetaController.atGoal()) thetaVelocity = 0.0;
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    m_swerve.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    m_swerve.stop();
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }
}
