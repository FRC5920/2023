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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Joystick.ProcessedXboxController;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class zTarget extends CommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  private Swerve s_Swerve;
  private ProcessedXboxController controller;
  private static final double SwerveP = 0.8;
  private static final double SwerveI = 0.0;
  private static final double SwervekD = 0.0;

  PhotonCamera TargetingCamera;
  GameTarget zTargetWhat;
  private final PIDController omegaController = new PIDController(SwerveP, SwerveI, SwervekD);

  public zTarget(
      GameTarget TargetWhat,
      PhotonCamera camera,
      Swerve s_Swerve,
      JoystickSubsystem joystickSubsystem,
      boolean fieldRelative,
      boolean openLoop) {

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.controller = joystickSubsystem.driverController;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.TargetingCamera = camera;
    this.zTargetWhat = TargetWhat;

    omegaController.setTolerance(Units.degreesToRadians(5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TargetingCamera.setPipelineIndex(zTargetWhat.PipelineIndex);
    TargetingCamera.setPipelineIndex(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get translation and rotation from the joystick controller
    double yAxis = -controller.getLeftY();
    double xAxis = -controller.getLeftX();
    double rotationRad = -controller.getRightX();

    PhotonPipelineResult result = TargetingCamera.getLatestResult();

    // If vision has acquired a target, we will overwrite the rotation with a value
    // that will rotate the bot toward the target
    if (result.hasTargets()) {

      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      // Current measurement is the yaw relative to the bot.
      // omegaController has a setpoint of zero.  So, it's trying to reduce the
      // yaw to zero.
      double angleToTargetDeg = result.getBestTarget().getYaw();
      double yawToTargetRad = Units.degreesToRadians(angleToTargetDeg);
      double zRotationRad = omegaController.calculate(yawToTargetRad);
      // zRotation = -1.0 * Math.signum(rotationRad) * Math.min(1.0, Math.abs(rotationRad));
      SmartDashboard.putNumber("zTarget/degreesToTarget", angleToTargetDeg);
      SmartDashboard.putNumber("zTarget/yawToTargetRad", yawToTargetRad);
      SmartDashboard.putNumber("zTarget/controllerRad", zRotationRad);

      rotationRad = zRotationRad; // NOTE: may need to cap this at a maximum...
    }

    translation = new Translation2d(yAxis, xAxis).times(RobotContainer.MaxSpeed);
    rotationRad *= RobotContainer.MaxRotate;
    SmartDashboard.putNumber("zTarget/commandedOutput", rotationRad);
    s_Swerve.drive(translation, rotationRad, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TargetingCamera.setPipelineIndex(GameTarget.DriveView.PipelineIndex());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
