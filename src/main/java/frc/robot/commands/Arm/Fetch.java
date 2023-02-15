// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.google.flatbuffers.Constants;

public class Fetch extends CommandBase {
  /** Creates a new Fetch. */
  double forwardSpeed;
  double rotationSpeed;
  public Fetch(Arm.GamePieceType FetchWhat) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // First calculate range
      double range =
              PhotonUtils.calculateDistanceToTargetMeters(
                      CAMERA_HEIGHT_METERS,
                      TARGET_HEIGHT_METERS,
                      CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(result.getBestTarget().getPitch()));

              // Use this range as the measurement we give to the PID controller.
              // -1.0 required to ensure positive PID controller effort _increases_ range
              forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            } 
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
