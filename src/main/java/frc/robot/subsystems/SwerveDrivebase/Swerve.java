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
package frc.robot.subsystems.SwerveDrivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveDrive.GyroIO;
import frc.lib.SwerveDrive.GyroInputsAutoLogged;
import frc.lib.SwerveDrive.SwerveModule;
import frc.lib.SwerveDrive.SwerveModuleIO;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private static final Rotation2d kAngleZero = new Rotation2d(0.0);
  private static final Rotation2d kAngle360 = Rotation2d.fromDegrees(360);

  /* Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
  private final SwerveDriveKinematics swerveKinematics;
  private final SwerveDriveOdometry swerveOdometry;
  private final SwerveModule[] mSwerveMods;
  private final GyroInputsAutoLogged m_gyroMeasurements;
  private final GyroIO m_gyroIO;

  /** Pose used during simulation */
  private Pose2d simOdometryPose = new Pose2d();

  /**
   * Creates an instance of the swerve module
   *
   * @param trackWidthMeters Track width of the drive train in meters
   * @param wheelBaseMeters Width of the wheel base in meters
   */
  public Swerve(
      double trackWidthMeters,
      double wheelBaseMeters,
      GyroIO gyroIO,
      SwerveModuleIO frontLeftIO,
      SwerveModuleIO frontRightIO,
      SwerveModuleIO rearLeftIO,
      SwerveModuleIO rearRightIO) {

    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBaseMeters / 2.0, trackWidthMeters / 2.0),
            new Translation2d(wheelBaseMeters / 2.0, (-1 * trackWidthMeters) / 2.0),
            new Translation2d((-1 * wheelBaseMeters) / 2.0, trackWidthMeters / 2.0),
            new Translation2d((-1 * wheelBaseMeters) / 2.0, (-1 * trackWidthMeters) / 2.0));

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(ModuleId.kFrontLeft.value, frontLeftIO),
          new SwerveModule(ModuleId.kFrontRight.value, frontRightIO),
          new SwerveModule(ModuleId.kRearLeft.value, rearLeftIO),
          new SwerveModule(ModuleId.kRearRight.value, rearRightIO)
        };

    m_gyroMeasurements = new GyroInputsAutoLogged();

    m_gyroIO = gyroIO;
    zeroGyro();

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getYaw(), getModulePositions());
  }

  /**
   * Moves the swerve drive to a given translation and rotation
   *
   * @param translation Vector representing the direction to travel
   * @param rotation Rotation to apply to the drive base
   * @param fieldRelative true if translation and rotation are given as field-relative
   * @param isOpenLoop true to use open-loop control when setting motor speeds; else false to enable
   *     closed-loop control
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    double x = translation.getX();
    double y = translation.getY();

    // Calculate chassis speeds from the given translation and rotation
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw())
            : new ChassisSpeeds(x, y, rotation);

    // Calculate new desired swerve module states from the chassis speeds
    SwerveModuleState[] newModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        newModuleStates, Constants.SwerveDrivebaseConstants.maxSpeed);

    // Apply the new desired module states
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(newModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /** Stops swerve drive motion */
  public void stop() {
    drive(new Translation2d(0, 0).times(0), 0, true, true);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.SwerveDrivebaseConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /** Returns the present pose */
  public Pose2d getPose() {
    return (RobotBase.isReal()) ? swerveOdometry.getPoseMeters() : simOdometryPose;
  }

  /** Resets odometry */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /** Returns swerve module states */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /** Returns swerve module positions */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /** Zeros the gyro */
  public void zeroGyro() {
    m_gyroMeasurements.yawRad = kAngleZero.getRadians();
    m_gyroIO.setYaw(kAngleZero);
  }

  /** Returns the yaw measurement */
  public Rotation2d getYaw() {
    Rotation2d yaw = Rotation2d.fromRadians(m_gyroMeasurements.yawRad);
    return (Constants.SwerveDrivebaseConstants.invertGyro) ? yaw.minus(kAngle360) : yaw;
  }

  /** Returns the roll measurement */
  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(m_gyroMeasurements.rollRad);
  }

  /** Returns the pitch measurement */
  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(m_gyroMeasurements.pitchRad);
  }

  /** Resets swerve module angles */
  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /** Executed periodically to service the subsystem */
  @Override
  public void periodic() {
    m_gyroIO.updateInputs(m_gyroMeasurements);
    Logger.getInstance().processInputs("Drive/Gyro", m_gyroMeasurements);
    for (SwerveModule module : mSwerveMods) {
      module.updateLoggedInputs();
    }

    swerveOdometry.update(getYaw(), getModulePositions());

    if (RobotBase.isSimulation()) {
      SwerveModuleState[] measuredStates =
          new SwerveModuleState[] {
            mSwerveMods[ModuleId.kFrontLeft.value].getState(),
            mSwerveMods[ModuleId.kFrontRight.value].getState(),
            mSwerveMods[ModuleId.kRearLeft.value].getState(),
            mSwerveMods[ModuleId.kFrontLeft.value].getState()
          };
      ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(measuredStates);
      simOdometryPose =
          simOdometryPose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * .02,
                  speeds.vyMetersPerSecond * .02,
                  speeds.omegaRadiansPerSecond * .02));
    }

    /*
    SmartDashboard.putNumber("Gyro", getYaw());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("Pitch", getPitch());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAngle().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    */
  }

  public SwerveModuleIO.SwerveModuleIOTelemetry getIOTelemetry(ModuleId module) {
    return mSwerveMods[module.value].getIOTelemetry();
  }

  /** Swerve module ID's */
  public enum ModuleId {
    kFrontLeft(0),
    kFrontRight(1),
    kRearLeft(2),
    kRearRight(3);

    public final int value;

    private ModuleId(int id) {
      value = id;
    }

    /**
     * @return the human-readable name of the module ID
     */
    @Override
    public String toString() {
      return this.name().substring(1); // Strip 'k' prefix from name
    }
  };
}
