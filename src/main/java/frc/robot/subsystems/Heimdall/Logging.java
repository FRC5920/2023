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
package frc.robot.subsystems.Heimdall;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Helper classes for AdvantageKit logging of inputs */
public class Logging {

  public static class Translation3dLoggableInput implements LoggableInputs {
    private final String logPrefix;
    public Translation3d value;

    public Translation3dLoggableInput(String prefix) {
      logPrefix = prefix + "/Translation3d";
      value = new Translation3d();
    }

    @Override
    public void toLog(LogTable table) {
      table.put((logPrefix + "/x"), value.getX());
      table.put((logPrefix + "/y"), value.getY());
      table.put((logPrefix + "/z"), value.getZ());
    }

    @Override
    public void fromLog(LogTable table) {
      value =
          new Translation3d(
              table.getDouble((logPrefix + "/x"), 0.0),
              table.getDouble((logPrefix + "/y"), 0.0),
              table.getDouble((logPrefix + "/z"), 0.0));
    }

    public Translation3dLoggableInput clone() {
      Translation3dLoggableInput copy = new Translation3dLoggableInput(this.logPrefix);
      copy.value = new Translation3d(value.getX(), value.getY(), value.getZ());
      return copy;
    }
  }

  public static class Translation2dLoggableInput implements LoggableInputs {
    private final String logPrefix;
    public Translation2d value;

    public Translation2dLoggableInput(String prefix) {
      logPrefix = prefix + "/Translation2d";
      value = new Translation2d();
    }

    @Override
    public void toLog(LogTable table) {
      table.put((logPrefix + "/x"), value.getX());
      table.put((logPrefix + "/y"), value.getY());
    }

    @Override
    public void fromLog(LogTable table) {
      value =
          new Translation2d(
              table.getDouble((logPrefix + "/x"), 0.0), table.getDouble((logPrefix + "/y"), 0.0));
    }

    public Translation2dLoggableInput clone() {
      Translation2dLoggableInput copy = new Translation2dLoggableInput(this.logPrefix);
      copy.value = new Translation2d(value.getX(), value.getY());
      return copy;
    }
  }

  public static class QuaternionLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Quaternion value;

    public QuaternionLoggableInput(String prefix) {
      logPrefix = prefix + "/Quaternion";
      value = new Quaternion();
    }

    @Override
    public void toLog(LogTable table) {
      table.put((logPrefix + "/W"), value.getW());
      table.put((logPrefix + "/X"), value.getX());
      table.put((logPrefix + "/Y"), value.getY());
      table.put((logPrefix + "/Z"), value.getZ());
    }

    @Override
    public void fromLog(LogTable table) {
      value =
          new Quaternion(
              table.getDouble((logPrefix + "/W"), 0.0),
              table.getDouble((logPrefix + "/X"), 0.0),
              table.getDouble((logPrefix + "/Y"), 0.0),
              table.getDouble((logPrefix + "/Z"), 0.0));
    }

    public QuaternionLoggableInput clone() {
      QuaternionLoggableInput copy = new QuaternionLoggableInput(this.logPrefix);
      copy.value = new Quaternion(value.getW(), value.getX(), value.getY(), value.getZ());
      return copy;
    }
  }

  public static class Rotation3dLoggableInput implements LoggableInputs {
    public final String logPrefix;
    private QuaternionLoggableInput quaternionInput;
    public Rotation3d value;

    public Rotation3dLoggableInput(String prefix) {
      logPrefix = prefix + "/Rotation3d";
      quaternionInput = new QuaternionLoggableInput(logPrefix);
      value = new Rotation3d();
    }

    @Override
    public void toLog(LogTable table) {
      quaternionInput.value = value.getQuaternion();
      quaternionInput.toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
      quaternionInput.fromLog(table);
      value = new Rotation3d(quaternionInput.value);
    }

    public Rotation3dLoggableInput clone() {
      Rotation3dLoggableInput copy = new Rotation3dLoggableInput(this.logPrefix);
      copy.quaternionInput = quaternionInput.clone();
      copy.value = new Rotation3d(quaternionInput.value);
      return copy;
    }
  }

  public static class Transform3dLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Translation3dLoggableInput translation3d;
    public Rotation3dLoggableInput rotation3d;

    public Transform3dLoggableInput(String prefix) {
      logPrefix = prefix + "/Transform3d";
      translation3d = new Translation3dLoggableInput(logPrefix);
      rotation3d = new Rotation3dLoggableInput(logPrefix);
    }

    public void update(Transform3d input) {
      translation3d.value = input.getTranslation();
      rotation3d.value = input.getRotation();
    }

    @Override
    public void toLog(LogTable table) {
      translation3d.toLog(table);
      rotation3d.toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
      translation3d.fromLog(table);
      rotation3d.fromLog(table);
    }

    public Transform3dLoggableInput clone() {
      Transform3dLoggableInput copy = new Transform3dLoggableInput(this.logPrefix);
      copy.rotation3d = this.rotation3d.clone();
      copy.translation3d = this.translation3d.clone();
      return copy;
    }
  }

  public static class Pose3dLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Translation3dLoggableInput translation3d;
    public Rotation3dLoggableInput rotation3d;

    public Pose3dLoggableInput(String prefix) {
      logPrefix = prefix + "/Pose3d";
      translation3d = new Translation3dLoggableInput(logPrefix);
      rotation3d = new Rotation3dLoggableInput(logPrefix);
    }

    public void update(Pose3d input) {
      translation3d.value = input.getTranslation();
      rotation3d.value = input.getRotation();
    }

    @Override
    public void toLog(LogTable table) {
      translation3d.toLog(table);
      rotation3d.toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
      translation3d.fromLog(table);
      rotation3d.fromLog(table);
    }

    public Pose3dLoggableInput clone() {
      Pose3dLoggableInput copy = new Pose3dLoggableInput(this.logPrefix);
      copy.rotation3d = this.rotation3d.clone();
      copy.translation3d = this.translation3d.clone();
      return copy;
    }
  }
}
