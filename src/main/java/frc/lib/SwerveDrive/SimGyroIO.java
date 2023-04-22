////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023-6328 FIRST and other WPILib contributors.
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
package frc.lib.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** Implementation of a simulated Gyro device */
public class SimGyroIO implements GyroIO {
  private static final Rotation2d kZeroRot = new Rotation2d();

  /** Pose used to simulate the gyro's yaw */
  private Pose2d m_simPose = new Pose2d();
  /** The simulated gyro's yaw velocity */
  private Rotation2d m_yawVelocity = kZeroRot;
  /** Timer used to calculate yaw velocity */
  private final Timer m_timer = new Timer();
  /** Flag indicating the first call to calculate() */
  private boolean m_isInitialCalculation = false;

  public SimGyroIO() {
    if (!RobotBase.isSimulation()) {
      throw new RuntimeException("SimGyroIO can only be used in simulation mode!");
    }
  }

  /** Set the simulated yaw */
  public void setYaw(Rotation2d angle) {
    m_simPose = new Pose2d(m_simPose.getTranslation(), angle);
    m_yawVelocity = kZeroRot;
  }

  /** Get measurements from the simulated Gyro */
  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.pitch = kZeroRot;
    inputs.roll = kZeroRot;
    inputs.yaw = m_simPose.getRotation();
    inputs.pitchVelocity = kZeroRot;
    inputs.rollVelocity = kZeroRot;
    inputs.yawVelocity = m_yawVelocity;
  }

  /**
   * This function must be called from the robot's periodic loop to calculate simulated Gyro values
   */
  public void calculate(double xVelocity, double yVelocity, double omegaRadPerSec) {
    // Get time elapsed since
    double dt = (m_isInitialCalculation) ? 1.0 : m_timer.get();
    m_isInitialCalculation = false;
    m_timer.restart();

    double lastYawRad = m_simPose.getRotation().getRadians();

    // calculate yaw
    final double kYawScale = 0.02;
    m_simPose =
        m_simPose.exp(
            new Twist2d(xVelocity * kYawScale, yVelocity * kYawScale, omegaRadPerSec * kYawScale));

    // calculate yaw velocity
    double deltaYaw = (lastYawRad - m_simPose.getRotation().getRadians()) / dt;
    m_yawVelocity = new Rotation2d(deltaYaw);
  }
}
