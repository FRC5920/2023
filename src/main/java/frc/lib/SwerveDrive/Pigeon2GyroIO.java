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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/** IO implementation for Pigeon2 */
public class Pigeon2GyroIO implements GyroIO {
  private final WPI_Pigeon2 pigeon;

  public Pigeon2GyroIO(int canID, String canBus) {
    pigeon = new WPI_Pigeon2(canID, canBus);
    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    pigeon.setYaw(0.0);
  }

  public void setYaw(Rotation2d angle) {
    double degrees = angle.getDegrees();
    pigeon.setYaw(degrees);
  }

  private static final int kYawIndex = 0;
  private static final int kPitchIndex = 1;
  private static final int kRollIndex = 2;
  private static final int kXAxisIndex = 0;
  private static final int kYAxisIndex = 1;
  private static final int kZAxisIndex = 2;
  /** Obtain measurements from the Pigeon2 */
  @Override
  public void updateInputs(GyroInputs OUTmeasurements) {
    double[] yprDegrees = new double[3];
    double[] xyzDegreesPerSec = new double[3];
    pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDegreesPerSec);
    OUTmeasurements.isConnected = pigeon.getLastError().equals(ErrorCode.OK);
    OUTmeasurements.roll = Rotation2d.fromDegrees(yprDegrees[kPitchIndex]);
    OUTmeasurements.pitch = Rotation2d.fromDegrees(-yprDegrees[kRollIndex]);
    OUTmeasurements.yaw = Rotation2d.fromDegrees(yprDegrees[kYawIndex]);
    OUTmeasurements.rollVelocity = Rotation2d.fromDegrees(xyzDegreesPerSec[kXAxisIndex]);
    OUTmeasurements.pitchVelocity = Rotation2d.fromDegrees(-xyzDegreesPerSec[kYAxisIndex]);
    OUTmeasurements.yawVelocity = Rotation2d.fromDegrees(xyzDegreesPerSec[kZAxisIndex]);
  }
}
