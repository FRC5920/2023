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
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.function.Supplier;

public class Drive2Pose extends CommandBase {
  private final Swerve s_swerve;
  private final Supplier<Pose2d> poseSupplier;
  private final double driveKp = 0.1;
  private final double driveKd = 0;
  private final double thetaKp = 10;
  private final double thetaKd = 0;
  private final double driveMaxVelocity = 3.5;
  private final double driveMaxAcceleration = 11;
  private final double thetaMaxVelocity = Units.degreesToRadians(360.0);
  private final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
  private final double driveTolerance = 0.05;
  private final double thetaTolerance = Units.degreesToRadians(10.0);

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  /** Drives to the specified pose under full software control. */
  public Drive2Pose(Swerve s_swerve, Pose2d pose) {
    this(s_swerve, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public Drive2Pose(Swerve s_swerve, Supplier<Pose2d> poseSupplier) {
    this.s_swerve = s_swerve;
    this.poseSupplier = poseSupplier;
    addRequirements(s_swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = s_swerve.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    driveController.setP(driveKp);
    driveController.setD(driveKd);
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    driveController.setTolerance(driveTolerance);
    thetaController.setP(thetaKp);
    thetaController.setD(thetaKd);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    thetaController.setTolerance(thetaTolerance);

    // Get current and target pose
    var currentPose = s_swerve.getPose();
    var targetPose = poseSupplier.get();

    // Command speeds
    double driveVelocityScalar =
        driveController.calculate(
            currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()), 0.0);
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
    s_swerve.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    s_swerve.stop();
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }
}
