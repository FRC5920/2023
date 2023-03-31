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
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.SwerveDrive.COTSFalconSwerveConstants;
import frc.lib.SwerveDrive.SwerveModuleConstants;
import java.util.*;
import org.littletonrobotics.junction.LoggedRobot;

public final class Constants {

  public static final double robotPeriodSec = LoggedRobot.defaultPeriodSecs;

  /** Set this to true to enable logging with AdvantageKit */
  public static final boolean kLoggingIsEnabled = false;

  /** Set the value of logPlaybackIsEnabled to true when replaying log files */
  public static final boolean kLogPlaybackIsEnabled = false;

  /** robotType indicates the type of bot this code applies to */
  public static final RobotType kRobotType = RobotType.PrototypeBot;

  /** Set to true when tuning or characterizing the robot */
  public static final boolean tuningMode = false;

  /** Returns the present robot mode */
  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : (kLogPlaybackIsEnabled ? Mode.REPLAY : Mode.SIM);
  }

  public static class DriverConstants {
    public static final int kControllerPort = 0;
    public static final double stickDeadband = 0.1;
  }

  public static class OperatorConstants {
    public static final int kControllerPort = 1;
    public static final double stickDeadband = 0.1;
  }

  public static class ArmConstants {
    public static final int kArmYMotorMasterPort = 7;
    public static final int kArmYMotorSlavePort = 6;
    public static final int kHandFrontRollerPort = 5;
    public static final int kHandBackRollerPort = 20;
    public static final int kArmExtenderPort = 8;

    public static final int kArmStoredPosition = 0;
    public static final int kArmIntakePosition = 500;
    public static final int kArmPlaceHighPosition = 2500;
    public static final int kArmPlaceMiddlePosition = 2600;
    public static final int kArmRetracted = 0;
    public static final int kArmExtendedHigh = 4000;
    public static final int kArmExtendedMiddle = 3000;
    public static final int kArmExtenderPIDLoopIdx = 0;
    public static final int kArmExtenderTimeoutMs = 50;
    public static final double kArmExtenderFF = 0;
    public static final double kArmExtenderP = 0;
    public static final double kArmExtenderI = 0;
    public static final double kArmExtenderD = 0;
    public static final double kArmExtenderIz = 0;
    public static final int kArmYPIDLoopIdx = 0;
    public static final int kArmYTimeoutMs = 50;
    public static final double kArmYFF = 0;
    public static final double kArmYP = 0;
    public static final double kArmYI = 0;
    public static final double kArmYD = 0;
    public static final double kArmYIz = 0;
    public static final long kDropWaitTime = 5000;
    public static final long kIntakeWaitTime = 5000;
    public static final double kFetchAngularP = 0.1;
    public static final double kFetchAngularD = 0.0;
  }

  public static final class SwerveDrivebaseConstants {
    public static final int pigeonID = 41;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants
        // NOTE: this must be tuned to specific robot
        chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(18.75); // /stock 21.73
    public static final double wheelBase = Units.inchesToMeters(18.75); // stock 21.73
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /**
     * Swerve drive motors will ramp from zero to maximum output in the time (in milliseconds) given
     * by this constant.
     */
    public static final double kDriveMotorRampMilliseconds = 200.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // NOTE: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.15763 / 12); // default: 0.32
    public static final double driveKV = (2.2961 / 12); // default: 1.51
    public static final double driveKA = (0.27127 / 12); // default: 0.27

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed =
        4.97; // NOTE: This must be tuned to specific robot Stock 4.5
    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // NOTE: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(287.8);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(112.9);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.6);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(151.00);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final
  class AutoConstants { // NOTE: The below constants are used in the swerve example auto, and must
    // be tuned
    // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class PneumaticsConstants {
    public static final int kPDHCAN = 62;
    public static final int kArmLeftRotatorPort = 0;
    public static final int kArmRightRotatorPort = 1;
  }

  public static class VisionConstants {

    public static final double fiducialAmbiguityLimit = 0.2;
    /** Physical location of the camera on the robot, relative to the center of the robot. */
    // TODO: get the actual location of the tag camera to the robot transform
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(-0.220625, 0.0, 0.1725), new Rotation3d(0, 0, Math.toRadians(0)));

    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
    static final String TagCameraName = "Heimdall_Tag_Camera";
    static final String ArmCameraName = "HD_USB_Camera";
    // pipeline indexes
    public static final int kConePipelineIndex = 1;
    public static final int kCubePipelineIndex = 0;
    public static final int kDriverCameraPipelineIndex = -1;
  }

  /** A map of directories where log files should be stored */
  public static final Map<RobotType, String> logDirectories =
      Map.of(RobotType.PrototypeBot, "/media/sda2/");

  /////////////////////////////////////////////////////////////////////////////
  /** Robot types */
  public enum RobotType {
    /** Prototype robot */
    PrototypeBot,

    /** Competition robot */
    CompetitionBot;

    /** Get the human-readable name of the robot type */
    @Override
    public String toString() {
      return this.name();
    }
  };

  /////////////////////////////////////////////////////////////////////////////
  /** Robot types */
  public enum Mode {
    /** Running on real robot hardware */
    REAL,

    /** Replaying data from a robot run */
    REPLAY,

    /** Running in a robot simulation */
    SIM;

    /** Get the human-readable name of the robot type */
    @Override
    public String toString() {
      return this.name();
    }
  };

  ///////////////
  public enum GameTarget {
    Cube(0),
    AprilTag2D(1);

    public final int PipelineIndex;

    private GameTarget(int count) {
      this.PipelineIndex = count;
    }

    public int PipelineIndex() {
      return PipelineIndex;
    }
  }
}
