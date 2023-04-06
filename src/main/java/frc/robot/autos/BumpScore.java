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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.utility.BotLogger.BotLog;
import frc.lib.utility.PIDGains;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.subsystems.SwerveDrivebase.Swerve;

/**
 * This command provides the following behavior: From an initial Grid position, the robot will drive
 * forward at high acceleration to an intermediate position (Waypoint A), causing a piece of cargo
 * loaded on the edge of the back of the robot to roll off. Then, it will back up to the grid to
 * push the cargo in.
 */
public class BumpScore extends SequentialCommandGroup {
  /** Tolerance for position */
  public static final double kPositionTolerance = 0.05;

  /** PID gains for position */
  public static final PIDGains kPositionPIDGains = new PIDGains(8, 0, 0.2);

  /** Default gains applied to controlling position */
  public static final PIDGains kRotationGains = new PIDGains(10, 0, 0.2);

  /** Default max velocity used for position constraints */
  public static final double kMaxVelocity = 6.0;
  /** Default max acceleration used for position constraints */
  public static final double kMaxAcceleration = 11;
  /** Default control constraints used for the position controller */
  public static final TrapezoidProfile.Constraints kPositionConstraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

  /** Creates a new BumpScore. */
  public BumpScore(Grids.ScoringPosition scoringPosition, Swerve swerveSubsystem) {
    Pose2d initialPose = scoringPosition.getPose();
    double initialX = initialPose.getTranslation().getX();
    double initialY = initialPose.getTranslation().getY();
    Rotation2d initialRot = initialPose.getRotation();

    boolean isBlueAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    double waypointAOffset = 0.5 * (isBlueAlliance ? 1.0 : -1.0);
    double xA = initialX + waypointAOffset;
    Pose2d waypointA = new Pose2d(new Translation2d(xA, initialY), initialRot);
    BotLog.Debugf("Drive to Waypoint A: x=%.2f y=%.2f\n", xA, initialY);

    double offsetTowardGrid = 0.0 * (isBlueAlliance ? -1.0 : 1.0);
    double xBackup = initialX + offsetTowardGrid;
    Pose2d backupWaypoint = new Pose2d(new Translation2d(xBackup, initialY), initialRot);
    BotLog.Debugf("Drive back to: x=%.2f y=%.2f\n", xBackup, initialY);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveToWaypoint(
            swerveSubsystem,
            waypointA,
            kPositionTolerance,
            DriveToWaypoint.kDefaultRotationToleranceRadians,
            kPositionPIDGains,
            kRotationGains,
            kPositionConstraints,
            DriveToWaypoint.kDefaultRotationConstraints),
        new DriveToWaypoint(
                swerveSubsystem,
                backupWaypoint,
                kPositionTolerance,
                DriveToWaypoint.kDefaultRotationToleranceRadians,
                kPositionPIDGains,
                kRotationGains,
                kPositionConstraints,
                DriveToWaypoint.kDefaultRotationConstraints)
            .withTimeout(2.0));
  }
}
