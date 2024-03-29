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

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveDrive.GyroIO;
import frc.lib.SwerveDrive.GyroIO.GyroInputs;
import frc.lib.SwerveDrive.SimGyroIO;
import frc.lib.SwerveDrive.SwerveModule;
import frc.lib.SwerveDrive.SwerveModuleIO;
import frc.lib.utility.BotLogger.BotLog;
import frc.robot.Constants;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;

public class Swerve extends SubsystemBase {
  /** Set to true to enable a dashboard tab for the Swerve subsystem */
  public static final boolean kDashboardTabIsEnabled = true;

  private static final Rotation2d kAngleZero = new Rotation2d(0.0);
  private static final Rotation2d kAngle360 = Rotation2d.fromDegrees(360);

  /* Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
  private final SwerveDriveKinematics swerveKinematics;
  public final SwerveDrivePoseEstimator swervePoseEstimator;
  private final SwerveModule[] mSwerveMods;
  private final GyroInputs m_gyroMeasurements;
  private final GyroIO m_gyroIO;
  private ChassisSpeeds m_ChassisSpeeds;

  /** Dashboard tab displayed in Shuffleboard */
  private final SwerveDashboardTab m_dashboardTab;

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

    m_dashboardTab = (kDashboardTabIsEnabled) ? new SwerveDashboardTab() : null;

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

    m_gyroMeasurements = new GyroInputs("Swerve/GyroInputs/");

    m_gyroIO = gyroIO;
    zeroGyro();

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(3.0);
    resetModulesToAbsolute();
    Timer.delay(3.0);
    resetModulesToAbsolute();

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

    m_ChassisSpeeds = new ChassisSpeeds();
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
    m_ChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw())
            : new ChassisSpeeds(x, y, rotation);

    // Calculate new desired swerve module states from the chassis speeds
    SwerveModuleState[] newModuleStates = swerveKinematics.toSwerveModuleStates(m_ChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        newModuleStates, Constants.SwerveDrivebaseConstants.maxSpeed);

    // Apply the new desired module states
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(newModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {

    m_ChassisSpeeds = speeds;

    // Calculate new desired swerve module states from the chassis speeds
    SwerveModuleState[] newModuleStates = swerveKinematics.toSwerveModuleStates(m_ChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        newModuleStates, Constants.SwerveDrivebaseConstants.maxSpeed);

    // Apply the new desired module states
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(newModuleStates[mod.moduleNumber], false);
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

  /** Sets all swerve wheels to a direction relative to the bot */
  /*
  private void setWheelPreset(WheelPreset preset) {
    SwerveModuleState desiredStates[] = getModuleStates();
    for (int idx = 0; idx < preset.angles.length; ++idx) {
      desiredStates[idx].angle = preset.angles[idx];
    }
    setModuleStates(desiredStates);
  }
  */

  /** Returns the present pose */
  public Pose2d getPose() {
    Pose2d pose;
    if (RobotBase.isReal()) {
      pose = swervePoseEstimator.getEstimatedPosition();
    } else {
      pose =
          new Pose2d(
              swervePoseEstimator.getEstimatedPosition().getTranslation(), m_gyroMeasurements.yaw);
    }
    return pose;
  }

  /** Resets odometry */
  public void resetOdometry(Pose2d pose) {
    BotLog.Infof(
        "Reset odometry: x=%.2fm, y=%.2fm, rot=%.2f deg",
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        pose.getRotation().getDegrees());
    swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
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

  /** Returns the current ChassisSpeeds */
  public ChassisSpeeds getChassisSpeeds() {
    return m_ChassisSpeeds;
  }

  /** Returns the swerve drive kinematics */
  public SwerveDriveKinematics getSwerveKinematics() {
    return swerveKinematics;
  }

  /** Zeros the gyro */
  public void zeroGyro() {
    resetGyro(kAngleZero);
    m_gyroIO.setYaw(kAngleZero);
  }

  /** Zeros the gyro, setting it to a specified angle */
  public void resetGyro(Rotation2d angle) {
    BotLog.Infof("Reset Gyro to %.2f deg", angle.getDegrees());

    m_gyroMeasurements.yaw = angle;
    m_gyroIO.setYaw(angle);
  }

  /** Returns the yaw measurement */
  public Rotation2d getYaw() {
    Rotation2d yaw = m_gyroMeasurements.yaw;
    return (Constants.SwerveDrivebaseConstants.invertGyro) ? yaw.minus(kAngle360) : yaw;
  }

  /** Returns the roll measurement */
  public Rotation2d getRoll() {
    return m_gyroMeasurements.roll;
  }

  /** Returns the pitch measurement */
  public Rotation2d getPitch() {
    return m_gyroMeasurements.pitch;
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

    // If running in simulation mode, recalculate the simulated gyro from swerve
    // module states
    if (RobotBase.isSimulation()) {
      // Calculate the simulated Gyro angle
      SwerveModuleState[] measuredStates =
          new SwerveModuleState[] {
            mSwerveMods[ModuleId.kFrontLeft.value].getState(),
            mSwerveMods[ModuleId.kFrontRight.value].getState(),
            mSwerveMods[ModuleId.kRearLeft.value].getState(),
            mSwerveMods[ModuleId.kFrontLeft.value].getState()
          };
      ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(measuredStates);
      ((SimGyroIO) m_gyroIO)
          .calculate(
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    // Update Gyro measurements
    m_gyroIO.updateInputs(m_gyroMeasurements);

    // Update swerve module measurements
    // Logger.getInstance().processInputs("Drive/Gyro", m_gyroMeasurements);
    for (SwerveModule module : mSwerveMods) {
      module.updateLoggedInputs();
    }

    swervePoseEstimator.update(getYaw(), getModulePositions());
  }

  public SwerveModuleIO.SwerveModuleIOTelemetry getIOTelemetry(ModuleId module) {
    return mSwerveMods[module.value].getIOTelemetry();
  }

  /** Register the subsystem's dashboard tab */
  public void registerDashboardTab(DashboardSubsystem dashboardSubsystem) {
    if (m_dashboardTab != null) {
      dashboardSubsystem.add(m_dashboardTab);
    }
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

  private static final double kWheelForward = 0.0;
  private static final double kWheelBackward = Math.PI;
  private static final double kWheelLeft = Math.PI / 2.0;
  private static final double kWheelRight = 1.5 * Math.PI;
  private static final double kWheelNorthWest = 0.75 * Math.PI;
  private static final double kWheelNorthEast = 0.25 * Math.PI;
  private static final double kWheelSouthWest = 1.25 * Math.PI;
  private static final double kWheelSouthEast = 1.75 * Math.PI;

  /** Preset directions for the swerve wheels */
  public enum WheelPreset {

    /** Wheels aligned for driving forward */
    Forward(new double[] {kWheelForward, kWheelForward, kWheelForward, kWheelForward}),
    /** Wheels aligned for driving backward */
    Reverse(new double[] {kWheelBackward, kWheelBackward, kWheelBackward, kWheelBackward}),
    /** Wheels aligned for slewing directly to the left */
    SlewLeft(new double[] {kWheelLeft, kWheelLeft, kWheelLeft, kWheelLeft}),
    /** Wheels aligned for slewing directly to the right */
    SlewRight(new double[] {kWheelRight, kWheelRight, kWheelRight, kWheelRight}),
    /** Wheels aligned at 45-degree angles for rotation in place to the right */
    RotateLeft(new double[] {kWheelSouthWest, kWheelNorthWest, kWheelSouthWest, kWheelNorthEast}),
    /** Wheels aligned at 45-degree angles for rotation in place to the left */
    RotateRight(new double[] {kWheelNorthEast, kWheelSouthEast, kWheelNorthWest, kWheelSouthWest});

    public final Rotation2d angles[];

    private WheelPreset(double _angles[]) {
      angles = new Rotation2d[_angles.length];
      for (int idx = 0; idx < angles.length; ++idx) {
        angles[idx] = new Rotation2d(_angles[idx]);
      }
    }
  }
}
