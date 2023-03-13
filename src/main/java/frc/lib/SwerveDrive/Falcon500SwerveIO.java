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
package frc.lib.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.robot.Constants;

/** Implementation of low-level Swerve module I/O routines for Falcon 500 motors */
public class Falcon500SwerveIO implements SwerveModuleIO {

  /** CANCoder used to measure the module's absolute angle */
  private final WPI_CANCoder m_angleEncoder;

  /** Falcon500 motor that drives the swerve module's angle */
  private final WPI_TalonFX m_angleMotor;

  /** Falcon500 motor that drives the swerve module's speed */
  private final WPI_TalonFX m_driveMotor;

  /** Calibration offset applied to the module's angle at the zero point */
  private final Rotation2d m_angleOffset;

  /** Feed-forward control applied to the module drive motor when operating in closed-loop mode */
  SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          Constants.SwerveDrivebaseConstants.driveKS,
          Constants.SwerveDrivebaseConstants.driveKV,
          Constants.SwerveDrivebaseConstants.driveKA);

  /**
   * Creates an instance of the moduleIO
   *
   * @param driveMotorCANId CAN bus ID of the module's drive motor
   * @param angleMotorCANId CAN bus ID of the module's angle motor
   * @param cancoderCANId CAN bus ID of the CANCoder used to measure the module's angle
   * @param ctreConfigs Configurations applied to motors and CANCoder
   */
  public Falcon500SwerveIO(
      int driveMotorCANId,
      int angleMotorCANId,
      int cancoderCANId,
      Rotation2d angleOffset,
      CTREConfigs ctreConfigs) {
    m_angleOffset = angleOffset;

    /* Angle Encoder Config */
    m_angleEncoder = new WPI_CANCoder(cancoderCANId, "SwerveCAN");
    configAngleEncoder(ctreConfigs);

    /* Angle Motor Config */
    m_angleMotor = new WPI_TalonFX(angleMotorCANId, "SwerveCAN");
    configAngleMotor(ctreConfigs);

    /* Drive Motor Config */
    m_driveMotor = new WPI_TalonFX(driveMotorCANId, "SwerveCAN");
    configDriveMotor(ctreConfigs);
  }

  /** Gather loggable values from the module */
  @Override
  public void updateLoggedInputs(SwerveModuleIOTelemetry inputs) {
    inputs.driveDistanceMeters = getDistance();
    inputs.driveSpeedMetersPerSecond = getSpeed();
    inputs.driveAppliedVolts = m_driveMotor.getMotorOutputVoltage();

    inputs.driveCurrentAmps = m_driveMotor.getStatorCurrent();
    inputs.driveTempCelcius = m_driveMotor.getTemperature();

    Rotation2d measuredAngleDegrees =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(
                m_angleMotor.getSelectedSensorPosition(),
                Constants.SwerveDrivebaseConstants.driveGearRatio));
    inputs.angleAbsolutePositionRad =
        MathUtil.angleModulus(measuredAngleDegrees.minus(m_angleOffset).getRadians());
    inputs.anglePositionRad = measuredAngleDegrees.getRadians();

    inputs.angleVelocityRadPerSec = 0.0; // TODO: populate this value from Falcon measurement

    inputs.angleAppliedVolts = m_angleMotor.getMotorOutputVoltage();
    inputs.angleCurrentAmps = m_angleMotor.getStatorCurrent();
    inputs.angleTempCelcius = m_angleMotor.getTemperature();
  }

  /**
   * Set the desired speed of the module
   *
   * @param speedMetersPerSecond desired speed in meters per second
   */
  @Override
  public void setSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = speedMetersPerSecond / Constants.SwerveDrivebaseConstants.maxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              speedMetersPerSecond,
              Constants.SwerveDrivebaseConstants.wheelCircumference,
              Constants.SwerveDrivebaseConstants.driveGearRatio);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          m_feedforward.calculate(speedMetersPerSecond));
    }
  }

  /**
   * Set the desired angle of the module
   *
   * @param angle angle as a Rotation2d object
   */
  @Override
  public void setAngle(Rotation2d angle) {
    m_angleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(
            angle.getDegrees(), Constants.SwerveDrivebaseConstants.angleGearRatio));
  }

  /**
   * Returns the current speed of the module
   *
   * @return the current speed of the module in meters per second
   */
  @Override
  public double getSpeed() {
    return Conversions.falconToMPS(
        m_driveMotor.getSelectedSensorVelocity(),
        Constants.SwerveDrivebaseConstants.wheelCircumference,
        Constants.SwerveDrivebaseConstants.driveGearRatio);
  }

  /**
   * Returns the current angle of the module
   *
   * @return the current angle of the module as a Rotation2d object
   */
  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_angleMotor.getSelectedSensorPosition(),
            Constants.SwerveDrivebaseConstants.angleGearRatio));
  }

  /**
   * Returns the current distance measurement from the module in meters
   *
   * @return the current distance measurement of the module in meters
   */
  @Override
  public double getDistance() {
    return Conversions.falconToMeters(
        m_driveMotor.getSelectedSensorPosition(),
        Constants.SwerveDrivebaseConstants.wheelCircumference,
        Constants.SwerveDrivebaseConstants.driveGearRatio);
  }

  /** Reset the swerve module angle to its zero position */
  @Override
  public void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - m_angleOffset.getDegrees(),
            Constants.SwerveDrivebaseConstants.angleGearRatio);
    m_angleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder(CTREConfigs ctreConfigs) {
    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor(CTREConfigs ctreConfigs) {
    m_angleMotor.configFactoryDefault();
    m_angleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
    m_angleMotor.setInverted(Constants.SwerveDrivebaseConstants.angleMotorInvert);
    m_angleMotor.setNeutralMode(Constants.SwerveDrivebaseConstants.angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor(CTREConfigs ctreConfigs) {
    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
    m_driveMotor.setInverted(Constants.SwerveDrivebaseConstants.driveMotorInvert);
    m_driveMotor.setNeutralMode(Constants.SwerveDrivebaseConstants.driveNeutralMode);
    m_driveMotor.setSelectedSensorPosition(0);
    m_driveMotor.configOpenloopRamp(
        0.001 * Constants.SwerveDrivebaseConstants.kDriveMotorRampMilliseconds);
    m_driveMotor.configClosedloopRamp(
        0.001 * Constants.SwerveDrivebaseConstants.kDriveMotorRampMilliseconds);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
  }
}
