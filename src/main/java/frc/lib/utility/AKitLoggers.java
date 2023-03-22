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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public class AKitLoggers {

  public static class Translation3dLoggableInput implements LoggableInputs {
    private final String logPrefix;
    public Translation3d value;

    public Translation3dLoggableInput(String prefix, Translation3d trans) {
      logPrefix = prefix + "/Translation3d";
      value = trans;
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
              table.getDouble((logPrefix + "/x"), Double.NaN),
              table.getDouble((logPrefix + "/y"), Double.NaN),
              table.getDouble((logPrefix + "/z"), Double.NaN));
    }

    public Translation3dLoggableInput clone() {
      Translation3dLoggableInput copy =
          new Translation3dLoggableInput(
              this.logPrefix, new Translation3d(value.getX(), value.getY(), value.getZ()));
      return copy;
    }
  }

  public static class QuaternionLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Quaternion value;

    public QuaternionLoggableInput(String prefix, Quaternion q) {
      logPrefix = prefix + "/Quaternion";
      value = q;
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
              table.getDouble((logPrefix + "/W"), Double.NaN),
              table.getDouble((logPrefix + "/X"), Double.NaN),
              table.getDouble((logPrefix + "/Y"), Double.NaN),
              table.getDouble((logPrefix + "/Z"), Double.NaN));
    }

    public QuaternionLoggableInput clone() {
      QuaternionLoggableInput copy =
          new QuaternionLoggableInput(
              this.logPrefix,
              new Quaternion(value.getW(), value.getX(), value.getY(), value.getZ()));
      return copy;
    }
  }

  public static class Rotation3dLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Rotation3d value;

    public Rotation3dLoggableInput(String prefix, Rotation3d rot) {
      logPrefix = prefix + "/Rotation3d";
      value = rot;
    }

    @Override
    public void toLog(LogTable table) {
      new QuaternionLoggableInput(logPrefix, value.getQuaternion()).toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
      QuaternionLoggableInput logInput = new QuaternionLoggableInput(logPrefix, new Quaternion());
      logInput.fromLog(table);
      value = new Rotation3d(logInput.value);
    }

    public Rotation3dLoggableInput clone() {
      Rotation3dLoggableInput copy =
          new Rotation3dLoggableInput(this.logPrefix, new Rotation3d(value.getQuaternion()));
      return copy;
    }
  }

  public static class Transform3dLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Translation3dLoggableInput translation3d;
    public Rotation3dLoggableInput rotation3d;

    public Transform3dLoggableInput(String prefix, Transform3d input) {
      logPrefix = prefix + "/Transform3d";
      translation3d = new Translation3dLoggableInput(logPrefix, input.getTranslation());
      rotation3d = new Rotation3dLoggableInput(logPrefix, input.getRotation());
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
      Transform3dLoggableInput copy =
          new Transform3dLoggableInput(this.logPrefix, new Transform3d());
      copy.rotation3d = this.rotation3d.clone();
      copy.translation3d = this.translation3d.clone();
      return copy;
    }
  }

  public static class Pose3dLoggableInput implements LoggableInputs {
    public final String logPrefix;
    public Translation3dLoggableInput translation3d;
    public Rotation3dLoggableInput rotation3d;

    public Pose3dLoggableInput(String prefix, Pose3d input) {
      logPrefix = prefix;
      translation3d = new Translation3dLoggableInput(logPrefix + "/Pose3d", input.getTranslation());
      rotation3d = new Rotation3dLoggableInput(logPrefix + "/Pose3d", input.getRotation());
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
      Pose3dLoggableInput copy = new Pose3dLoggableInput(this.logPrefix, new Pose3d());
      copy.rotation3d = this.rotation3d.clone();
      copy.translation3d = this.translation3d.clone();
      return copy;
    }
  }
}
