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
package frc.lib.utility;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GameTarget;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * ZTargeter implements Z-targeting: using vision subsystems to identify a gamepiece and producing a
 * recommended rotation to center on it
 */
public class ZTargeter {

  // Set this constant to true to send values to the dashboard for debugging
  public static final boolean kEnableDashboardDebug = true;

  /** Default proportional gain used for the rotation PID controller */
  public static final double kDefault_kP = 1.0;
  /** Default integral gain used for the rotation PID controller */
  public static final double kDefault_kI = 0.0;
  /** Default derivative gain used for the rotation PID controller */
  public static final double kDefault_kD = 0.0;

  /** Camera used to target the gamepiece */
  private final PhotonCamera m_camera;

  /** Type of gamepiece to target */
  private final GameTarget m_gamepieceType;

  /**
   * PID controller used to produce a control effort that will bring the rotation of the camera to
   * center on a detected gamepiece
   */
  private final PIDController omegaController;

  /**
   * Creates an instance of the targeter that uses a given camera to target a given type of
   * gamepiece
   *
   * @param TargetWhat The type of gamepiece to target
   * @param camera Camera used to locate and target the gamepiece
   * @param gains PID gains to use for converging on the target
   */
  public ZTargeter(GameTarget TargetWhat, PhotonCamera camera) {
    this(TargetWhat, camera, new PIDGains(kDefault_kP, kDefault_kI, kDefault_kD));
  }

  /**
   * Creates an instance of the targeter that uses a given camera to target a given type of
   * gamepiece
   *
   * @param TargetWhat The type of gamepiece to target
   * @param camera Camera used to locate and target the gamepiece
   * @param gains PID gains to use for converging on the target
   */
  public ZTargeter(GameTarget TargetWhat, PhotonCamera camera, PIDGains gains) {
    m_camera = camera;
    m_gamepieceType = TargetWhat;

    omegaController = new PIDController(gains.kP, gains.kI, gains.kD);
    omegaController.setTolerance(Units.degreesToRadians(5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setSetpoint(0);
  }

  /** This method must be called to initialize the camera used by the ZTargeter */
  public void initialize() {
    m_camera.setPipelineIndex(m_gamepieceType.PipelineIndex);
  }

  /**
   * This method is called regularly to process vision results from the ZTargeter's camera and get a
   * rotation needed to center on a target
   *
   * @return null if no target is detected; else a Rotation2d object indicating the direction and
   *     amount to rotate in order to center on a detected target
   */
  public Rotation2d getRotationToTarget() {
    Rotation2d result = null;

    PhotonPipelineResult pipelineResult = m_camera.getLatestResult();

    // If vision has acquired a target, we will overwrite the rotation with a value
    // that will rotate the bot toward the target
    if (pipelineResult.hasTargets()) {

      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      // Current measurement is the yaw relative to the bot.
      // omegaController has a setpoint of zero.  So, it's trying to reduce the
      // yaw to zero.
      double angleToTargetDeg = pipelineResult.getBestTarget().getYaw();
      double yawToTargetRad = Units.degreesToRadians(angleToTargetDeg);
      double zRotationRad = omegaController.calculate(yawToTargetRad);

      if (kEnableDashboardDebug) {
        SmartDashboard.putNumber("zTarget/degreesToTarget", angleToTargetDeg);
        SmartDashboard.putNumber("zTarget/yawToTargetRad", yawToTargetRad);
        SmartDashboard.putNumber("zTarget/controllerRad", zRotationRad);
      }

      result = new Rotation2d(zRotationRad);
    }

    return result;
  }
}
