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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Implementation of low-level Swerve module I/O routines for a simulated swerve module */
public class SimSwerveModuleIO implements SwerveModuleIO {

  private static final double kMaxAppliedVolts = 12.0;
  private static final double kMinAppliedVolds = kMaxAppliedVolts * -1.0;

  /** Module wheel radius used for simulation */
  private static final double kWheelCircumferenceMeters =
      Constants.SwerveDrivebaseConstants.wheelCircumference;

  /** Gear ratio used to simulate the drive motor */
  private static final double kDriveMotorGearRatio = 6.75;
  /** Moment of inertia (kgm^2) used to simulate the drive motor */
  private static final double kDriveMotorMomentOfInertia = 0.025;

  /** Gear ratio used to simulate the angle motor */
  private static final double kAngleMotorGearRatio = 150.0 / 7.0;

  /** Moment of inertia (kgm^2) used to simulate the angle motor */
  private static final double kAngleMotorMomentOfInertia = 0.004;

  private static final double kAnglePIDkP = 5.0;
  private static final double kAnglePIDkI = 0.0;
  private static final double kAnglePIDkD = 0.0;

  /** Simulation of a module drive motor using a flywheel */
  private final FlywheelSim m_simDriveMotor =
      new FlywheelSim(DCMotor.getFalcon500(1), kDriveMotorGearRatio, kDriveMotorMomentOfInertia);

  /** Simulation of a module angle motor using a flywheel */
  private final FlywheelSim m_simAngleMotor =
      new FlywheelSim(DCMotor.getFalcon500(1), kAngleMotorGearRatio, kAngleMotorMomentOfInertia);

  private final PIDController m_anglePID =
      new PIDController(kAnglePIDkP, kAnglePIDkI, kAnglePIDkD, Constants.robotPeriodSec);

  private double m_driveDistanceMeters = 0.0;
  private double m_angleRelativePositionRad = 0.0;
  private double m_angleAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double m_driveMotorAppliedVolts = 0.0;
  private double m_angleMotorAppliedVolts = 0.0;

  public double m_requestedSpeedMetersPerSec = 0.0;
  public double m_requestedAngleDegrees = 0.0;

  /** Updates the set of loggable inputs */
  public void updateLoggedInputs(SwerveModuleIOTelemetry inputs) {
    // Update the state of the simulated drive motor
    m_simDriveMotor.update(Constants.robotPeriodSec);

    // Update the state of the simulated angle motor and its associated PID controller
    double volts = m_anglePID.calculate(getAngle().getRadians());
    m_angleMotorAppliedVolts = MathUtil.clamp(volts, kMinAppliedVolds, kMaxAppliedVolts);
    m_simAngleMotor.setInputVoltage(m_angleMotorAppliedVolts);
    m_simAngleMotor.update(Constants.robotPeriodSec);

    // Calculate the distance driven during the current interval based on current velocity
    // and apply this delta to the driven distance measurement
    double speedMetersPerSec = getSpeed();
    double deltaMeters = speedMetersPerSec * Constants.robotPeriodSec;
    m_driveDistanceMeters += deltaMeters;

    inputs.driveSpeedMetersPerSecond = speedMetersPerSec;
    inputs.driveDistanceMeters = m_driveDistanceMeters;
    inputs.driveAppliedVolts = m_driveMotorAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_simDriveMotor.getCurrentDrawAmps());
    inputs.driveTempCelcius = 0.0;

    // Calculate current module angle
    double angleDiffRad = m_simAngleMotor.getAngularVelocityRadPerSec() * Constants.robotPeriodSec;
    m_angleRelativePositionRad += angleDiffRad;
    m_angleAbsolutePositionRad += angleDiffRad;

    inputs.anglePositionRad = m_angleRelativePositionRad;
    inputs.angleAbsolutePositionRad = m_angleAbsolutePositionRad;
    inputs.angleVelocityRadPerSec = m_simAngleMotor.getAngularVelocityRadPerSec();
    inputs.angleAppliedVolts = m_angleMotorAppliedVolts;
    inputs.angleCurrentAmps = Math.abs(m_simAngleMotor.getCurrentDrawAmps());
    inputs.angleTempCelcius = 0.0;

    inputs.requestedSpeedMetersPerSec = m_requestedSpeedMetersPerSec;
    inputs.requestedAngleDegrees = m_requestedAngleDegrees;
  }

  /**
   * Set the desired speed of the module
   *
   * @param speedMetersPerSecond desired speed in meters per second
   * @param isOpenLoop true to use open-loop control of speed; else false for closed-loop control
   */
  @Override
  public void setSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
    m_requestedSpeedMetersPerSec = speedMetersPerSecond;
    double m_driveMotorVelocityRadPerSec =
        speedMetersPerSecond * 2.0 * Math.PI / kWheelCircumferenceMeters;
    m_driveMotorAppliedVolts =
        MathUtil.clamp(m_driveMotorVelocityRadPerSec, kMinAppliedVolds, kMaxAppliedVolts);
    m_simDriveMotor.setInputVoltage(m_driveMotorAppliedVolts);
  }

  /**
   * Set the desired angle of the module
   *
   * @param angle angle as a Rotation2d object
   */
  @Override
  public void setAngle(Rotation2d angle) {
    m_requestedAngleDegrees = angle.getDegrees();
    double volts = m_anglePID.calculate(getAngle().getRadians(), angle.getRadians());
    m_angleMotorAppliedVolts = MathUtil.clamp(volts, kMinAppliedVolds, kMaxAppliedVolts);
    m_simAngleMotor.setInputVoltage(m_angleMotorAppliedVolts);
  }

  /**
   * Returns the current speed of the module
   *
   * @return the current speed of the module in meters per second
   */
  @Override
  public double getSpeed() {
    return m_simDriveMotor.getAngularVelocityRadPerSec()
        * kWheelCircumferenceMeters
        / (2.0 * Math.PI);
  }

  /**
   * Returns the current angle of the module
   *
   * @return the current angle of the module as a Rotation2d object
   */
  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_angleAbsolutePositionRad);
  }

  /**
   * Returns the current distance measurement from the module in meters
   *
   * @return the current distance measurement of the module in meters
   */
  @Override
  public double getDistance() {
    return m_driveDistanceMeters;
  }
}
