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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utility.ZTargeter;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.commands.Shooter.Acquire;
import frc.robot.commands.SimulationPrinter;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import org.photonvision.PhotonCamera;

/**
 * AutoZTargetedIntake is a command that automatically centers on a gamepiece, then drives forward
 * and intakes it
 */
public class AutoZTargetedIntake extends CommandBase {
  /** The speed to approach the target at */
  private static final double kAcquireApproachSpeedMetersPerSec = RobotContainer.MaxSpeed / 2.0;
  /** Angle tolerance when centering on the gamepiece */
  private static final double kAutoAcquireAngleToleranceDeg = 2.0;
  /** Swerve subsystem accessed by the command */
  private final Swerve m_swerveSubsystem;
  /** Z-targeting engine */
  private final ZTargeter m_zTargeter;
  /** Type of gamepiece being targeted */
  private GameTarget m_gamepieceType;

  /** Returns a command that drives with Z-targeting and intake engaged */
  public static CommandBase create(
      GameTarget gamepieceType,
      PhotonCamera camera,
      Swerve swerveSubsystem,
      ShooterPivotSubsystem shooterPivot,
      IntakeSubsystem intake) {
    return Commands.sequence(
        new SimulationPrinter(String.format("<AutoZTargetedIntake> started")),
        Commands.race(
            new AutoZTargetedIntake(gamepieceType, camera, swerveSubsystem),
            Acquire.acquireAndPark(shooterPivot, intake)));
  }

  /** Creates an instance of the command */
  public AutoZTargetedIntake(
      GameTarget gamepieceType, PhotonCamera camera, Swerve swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    m_gamepieceType = gamepieceType;
    m_zTargeter = new ZTargeter(gamepieceType, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_zTargeter.initialize(); // Initialize Z-targeting
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get translation and rotation from the joystick controller
    boolean isFieldRelative = false;
    double rotationRad = 0.0;

    // Get the rotation to a target.  Returns null if no target is found
    Rotation2d zRotation = m_zTargeter.getRotationToTarget();
    boolean targetExists = (zRotation != null);
    if (targetExists) {
      rotationRad = zRotation.getRadians();

      // If we're fetching a cube and the driver is pressing the right stick button and
      // our rotation is within an angle tolerance, then advance to grab the cube
      // if ((m_gamepieceType == GameTarget.Cube)
      //     && (m_controller.getRightStickButton())
      //     && (rotationRad < Units.degreesToRadians(kAutoAcquireAngleToleranceDeg))) {
      //   // Drive forward to grab the cube
      //   translation = new Translation2d(-1.0 * kAcquireApproachSpeedMetersPerSec, 0.0);
      //   isFieldRelative = false;
      // }

      //   if ((m_gamepieceType == GameTarget.Cube)
      //       && ((Math.abs(m_controller.getRightY()) > 0.1)
      //           || (Math.abs(m_controller.getRightX()) > 0.1))) {
      //     translation =
      //         new Translation2d(m_controller.getRightY(), m_controller.getRightX())
      //             .times(RobotContainer.MaxSpeed);
      //     isFieldRelative = false;
      //   } else {
      //     translation = new Translation2d(yAxis, xAxis).times(RobotContainer.MaxSpeed);
      //     isFieldRelative = true;
      //   }
    }

    // rotationRad *= RobotContainer.MaxRotate;
    // SmartDashboard.putNumber("zTarget/commandedOutput", rotationRad);
    // m_swerveSubsystem.drive(translation, rotationRad, isFieldRelative, m_openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
