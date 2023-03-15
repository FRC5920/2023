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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Joystick.ProcessedXboxController;
import frc.lib.utility.ZTargeter;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import org.photonvision.PhotonCamera;

public class DriveWithZTargeting extends CommandBase {

  private final boolean m_fieldRelative;
  private final boolean m_openLoop;
  private final Swerve m_swerveSubsystem;
  private final ProcessedXboxController m_controller;
  private final ZTargeter m_zTargeter;

  /** Returns a command that drives with Z-targeting and intake engaged */
  public static CommandBase zTargetDriveWithIntake(
      GameTarget gamepieceType,
      PhotonCamera camera,
      Swerve swerveSubsystem,
      JoystickSubsystem joystickSubsystem,
      boolean fieldRelative,
      boolean openLoop) {
    return Commands.race(
        new DriveWithZTargeting(
            gamepieceType, camera, swerveSubsystem, joystickSubsystem, fieldRelative, openLoop)
        // TODO: add acquire command
        );
  }

  public DriveWithZTargeting(
      GameTarget gamepieceType,
      PhotonCamera camera,
      Swerve swerveSubsystem,
      JoystickSubsystem joystickSubsystem,
      boolean fieldRelative,
      boolean openLoop) {

    m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    m_controller = joystickSubsystem.driverController;
    m_fieldRelative = fieldRelative;
    m_openLoop = openLoop;

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
    double yAxis = -m_controller.getLeftY();
    double xAxis = -m_controller.getLeftX();

    Rotation2d zRotation = m_zTargeter.getRotationToTarget();
    double rotationRad = (zRotation != null) ? zRotation.getRadians() : -m_controller.getRightX();

    Translation2d translation = new Translation2d(yAxis, xAxis).times(RobotContainer.MaxSpeed);
    rotationRad *= RobotContainer.MaxRotate;
    SmartDashboard.putNumber("zTarget/commandedOutput", rotationRad);
    m_swerveSubsystem.drive(translation, rotationRad, m_fieldRelative, m_openLoop);
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
