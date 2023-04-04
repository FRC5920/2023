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
package frc.robot.commands.zTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.utility.BotBoundary.PoseLimiter;
import frc.lib.utility.BotLogger.BotLog;
import frc.lib.utility.PIDGains;
import frc.lib.utility.ZTargeter;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.commands.Shooter.Acquire;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import org.photonvision.PhotonCamera;

public class AutoIntakeWithZTargeting extends SequentialCommandGroup {
  private static final double kApproachSpeedMetersPerSec = RobotContainer.MaxSpeed / 2.0;

  /** Returns a command that drives with Z-targeting and intake engaged */
  public AutoIntakeWithZTargeting(
      GameTarget gamepieceType,
      PhotonCamera camera,
      Swerve swerveSubsystem,
      ShooterPivotSubsystem shooterPivotSubsystem,
      IntakeSubsystem intakeSubsystem,
      PoseLimiter poseLimits) {

    addCommands(
        new BotLog.InfoPrintCommand("<AutoIntakeWithZTargeting> engaged"),
        Commands.race(
            new ZTargetAndDriveToGamepiece(
                gamepieceType, kApproachSpeedMetersPerSec, camera, swerveSubsystem, poseLimits),
            Acquire.acquireAndPark(shooterPivotSubsystem, intakeSubsystem)),
        // new IntakeGamepiece(intakeSubsystem)),
        new BotLog.InfoPrintCommand(String.format("<AutoIntakeWithZTargeting> finished")));
  }

  private static class ZTargetAndDriveToGamepiece extends CommandBase {
    private static final PIDGains kRotationControllerGains = new PIDGains(0.9, 0.0, 0.1);
    private static final double kRotationControllerToleranceRad = Units.degreesToRadians(5.0);

    private final GameTarget m_gamepieceType;
    private final Swerve m_swerveSubsystem;
    private final PoseLimiter m_poseLimits;
    private final ZTargeter m_zTargeter;
    private double m_approachSpeedMetersPerSec;
    private boolean m_targetDetected = false;
    private boolean m_lastTargetDetected = false;

    /**
     * Creates an instance of the command
     *
     * @param gamepieceType Type of gamepiece to Z-target
     * @param approachSpeedMetersPerSec Speed at which to approach the target
     * @param camera PhotonVision camera used for Z-targeting
     * @param swerveSubsystem Swerve subsystem used to move the bot
     * @param poseSupplier
     */
    public ZTargetAndDriveToGamepiece(
        GameTarget gamepieceType,
        double approachSpeedMetersPerSec,
        PhotonCamera camera,
        Swerve swerveSubsystem,
        PoseLimiter poseLimits) {
      m_gamepieceType = gamepieceType;
      m_approachSpeedMetersPerSec = approachSpeedMetersPerSec;
      m_swerveSubsystem = swerveSubsystem;
      addRequirements(swerveSubsystem);
      m_zTargeter =
          new ZTargeter(
              gamepieceType, camera, kRotationControllerGains, kRotationControllerToleranceRad);
      m_poseLimits = poseLimits;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if (RobotBase.isSimulation()) {
        BotLog.Infof(
            "<AutoZTargetAndDriveToGamepiece> engage Z tracking for %s\n", m_gamepieceType.name());
      }

      m_zTargeter.initialize(); // Initialize Z-targeting
      m_targetDetected = m_lastTargetDetected = false; // Initialize target detection history
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      // Use Z-targeter to get the rotation needed to center a target gamepiece.  This returns null
      // if no target is found
      Rotation2d zRotation = m_zTargeter.getRotationToTarget();

      // Print target detection
      m_lastTargetDetected = m_targetDetected;
      m_targetDetected = (zRotation != null);
      if (m_targetDetected && !m_lastTargetDetected) {
        BotLog.Infof("<AutoZTargetAndDriveToGamepiece> %s detected\n", m_gamepieceType.name());
      }

      Translation2d translationDelta = new Translation2d();
      double rotationDelta = 0.0;

      if (m_targetDetected) {
        // Get the rotation needed to align to bot with the target
        rotationDelta = zRotation.getRadians();

        // If the target is aligned and the current pose is not outside the given limits,
        // drive toward the target at the configured speed
        // SmartDashboard.putNumber(
        //    "AutoZTarget/alignError", m_zTargeter.getTargetAlignmentError().getDegrees());

        if (m_zTargeter.targetIsAligned()) {
          if (m_poseLimits.shouldLimitPose()) {
            BotLog.Infof("<AutoZTargetAndDriveToGamepiece> Pose limiter is preventing advance");
          } else {
            translationDelta = new Translation2d(-1.0 * m_approachSpeedMetersPerSec, 0.0);
          }
        }
      }

      // Drive open-loop, bot-relative
      // SmartDashboard.putNumber("AutoZTarget/Rot", rotationDelta);
      rotationDelta *= RobotContainer.MaxRotate;
      // SmartDashboard.putNumber("AutoZTarget/RotScaled", rotationDelta);
      m_swerveSubsystem.drive(translationDelta, rotationDelta, false, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      BotLog.Infof("<AutoZTargetAndDriveToGamepiece> interrupted");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
