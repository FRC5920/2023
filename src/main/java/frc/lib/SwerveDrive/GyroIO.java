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

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.utility.AKitLogging.Rotation2dLoggableInput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Base class for a Gyro device */
public interface GyroIO {

  /**
   * Sets the yaw value of the underlying Gyro implementation
   *
   * @param angle Angle to set yaw to
   */
  public void setYaw(Rotation2d angle);

  /**
   * Gathers measurements from the Gyro device
   *
   * @param [out] OUTmeasurements measurements to populate with values from the device
   */
  public void updateInputs(GyroInputs OUTmeasurements);

  /**
   * Measurements acquired from a Gyro device
   *
   * @remarks Measurement fields are given in accordance with WPILib coordinate system conventions
   *     documented at:
   *     https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
   *     <p>* The direction the robot is facing is the positive X-axis * The positive Y-axis points
   *     to the left of the robot * Positive values of theta (yaw) cause the bot to rotate in a
   *     counter-clockwise direction
   */
  public static class GyroInputs implements LoggableInputs {
    public String logPrefix;

    /** true if the Gyro is connected; else false */
    public boolean isConnected = false;
    /** Roll gives the rotation about the robot's X-axis in radians */
    public Rotation2d roll = new Rotation2d();
    /** Pitch gives the rotation about the robot's Y-axis in radians */
    public Rotation2d pitch = new Rotation2d();
    /** Yaw (a.k.a. theta) gives the rotation about the robot's Z-axis in radians */
    public Rotation2d yaw = new Rotation2d();
    /** The change in roll in radians per second */
    public Rotation2d rollVelocity = new Rotation2d();
    /** The change in pitch in radians per second */
    public Rotation2d pitchVelocity = new Rotation2d();
    /** The change in yaw/theta in radians per second */
    public Rotation2d yawVelocity = new Rotation2d();

    private Rotation2dLoggableInput rollInput;
    private Rotation2dLoggableInput pitchInput;
    private Rotation2dLoggableInput yawInput;
    private Rotation2dLoggableInput rollVelocityInput;
    private Rotation2dLoggableInput pitchVelocityInput;
    private Rotation2dLoggableInput yawVelocityInput;

    /** Creates an instance of the object and initializes measurements */
    public GyroInputs(String keyPrefix) {
      this.logPrefix = keyPrefix;
      rollInput = new Rotation2dLoggableInput(keyPrefix + "/roll");
      pitchInput = new Rotation2dLoggableInput(keyPrefix + "/pitch");
      yawInput = new Rotation2dLoggableInput(keyPrefix + "/yaw");
      rollVelocityInput = new Rotation2dLoggableInput(keyPrefix + "/rollVel");
      pitchVelocityInput = new Rotation2dLoggableInput(keyPrefix + "/pitchVel");
      yawVelocityInput = new Rotation2dLoggableInput(keyPrefix + "/yawVel");
    }

    /** Write measurements to a log table */
    @Override
    public void toLog(LogTable table) {
      rollInput.value = roll;
      pitchInput.value = pitch;
      yawInput.value = yaw;
      rollVelocityInput.value = rollVelocity;
      pitchVelocityInput.value = pitchVelocity;
      yawVelocityInput.value = yawVelocity;
      rollInput.toLog(table);
      pitchInput.toLog(table);
      yawInput.toLog(table);
      rollVelocityInput.toLog(table);
      pitchVelocityInput.toLog(table);
      yawVelocityInput.toLog(table);
    }

    /** Read measurements from a log table */
    @Override
    public void fromLog(LogTable table) {
      rollInput.fromLog(table);
      pitchInput.fromLog(table);
      yawInput.fromLog(table);
      rollVelocityInput.fromLog(table);
      pitchVelocityInput.fromLog(table);
      yawVelocityInput.fromLog(table);
      roll = rollInput.value;
      pitch = pitchInput.value;
      yaw = yawInput.value;
      rollVelocity = rollVelocityInput.value;
      pitchVelocity = pitchVelocityInput.value;
      yawVelocity = yawVelocityInput.value;
    }

    public GyroInputs clone() {
      GyroInputs copy = new GyroInputs(this.logPrefix);
      copy.pitch = this.pitch;
      copy.roll = this.roll;
      copy.yaw = this.yaw;
      copy.pitchVelocity = this.pitchVelocity;
      copy.rollVelocity = this.rollVelocity;
      copy.yawVelocity = this.yawVelocity;
      return copy;
    }
  }
}
