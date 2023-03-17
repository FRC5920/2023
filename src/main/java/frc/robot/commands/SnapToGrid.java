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
package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Joystick.ProcessedXboxController;
import frc.robot.autos.AutoConstants.BotDimensions;
import frc.robot.autos.AutoConstants.Grids;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.HashMap;

public class SnapToGrid extends CommandBase {
  /** Y values comprising grid lines */
  public static final double kGridYValues[] =
      new double[] {
        Grids.ScoringPosition.B.getPosition().getY(),
        Grids.ScoringPosition.E.getPosition().getY(),
        Grids.ScoringPosition.H.getPosition().getY(),
      };

  /** Distance from a grid line to capture */
  private static final double kCaptureRangeMeters = BotDimensions.kFootprintWidth;
  // (Grids.ScoringPosition.B.getPosition().getY() - Grids.ScoringPosition.A.getPosition().getY())
  //     / 2.0;

  private static final double kP = 0.8;
  private static final double kI = 0.0;
  private static final double kD = 0.08;
  private static final double kErrorTolerance = 0.02;

  private final double m_maxSpeed;
  private final double m_maxRotation;

  GridHelper m_gridHelper = new GridHelper(kGridYValues, kCaptureRangeMeters);

  private final boolean m_fieldRelative;
  private final boolean m_openLoop;

  private Swerve m_swerveSubsystem;
  private ProcessedXboxController m_controller;

  private Double m_snapValue = null;

  /** PID controller used to drive toward the snap value */
  private final PIDController m_snapPID = new PIDController(kP, kI, kD);

  private GridFieldLinesHelper m_gridFieldLineHelper;

  /** Creates an instance of the command */
  public SnapToGrid(
      Swerve swerveSubsystem,
      JoystickSubsystem joystickSubsystem,
      boolean fieldRelative,
      boolean openLoop,
      double maxSpeed,
      double maxRotation,
      Field2d field) {
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    m_maxSpeed = maxSpeed;
    m_maxRotation = maxRotation;
    m_controller = joystickSubsystem.driverController;
    m_fieldRelative = fieldRelative;
    m_openLoop = openLoop;
    m_snapPID.setTolerance(kErrorTolerance);
    m_gridFieldLineHelper = new GridFieldLinesHelper(field);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gridFieldLineHelper.display();
    SmartDashboard.putNumber("S2G-snapValue", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get values requested by the operator
    double deltaY = -m_controller.getLeftX();
    double deltaX = -m_controller.getLeftY();
    double deltaRot = -m_controller.getRightX();

    // Get the current Y coordinate
    Translation2d currentPosition = m_swerveSubsystem.getPose().getTranslation();
    double currentY = currentPosition.getY();

    // Determine if the bot is within range of a grid line
    Double snapValue = m_gridHelper.getNearestGridValue(currentY);
    if (snapValue != null) {
      // If snapping to a new value, reset the PID controller
      if (snapValue != m_snapValue) {
        m_snapValue = snapValue;
        m_snapPID.reset();
      }

      m_snapPID.setSetpoint(snapValue);
      // Calculate a speed value to drive the Y value to the nearest grid
      double snapDeltaY = m_snapPID.calculate(currentY, m_snapValue);
      deltaY = snapDeltaY;

      SmartDashboard.putNumber("S2G-snapValue", m_snapValue);
      SmartDashboard.putNumber("S2G-snapDeltaY", m_snapValue);
    } else {
      SmartDashboard.putNumber("S2G-snapValue", Double.NaN);
    }

    SmartDashboard.putNumber("S2G-poseX", currentPosition.getX());
    SmartDashboard.putNumber("S2G-poseY", currentPosition.getY());
    SmartDashboard.putNumber("S2G-deltaX", deltaX);
    SmartDashboard.putNumber("S2G-deltaY", deltaY);
    Translation2d translationSpeeds = new Translation2d(deltaX, deltaY).times(m_maxSpeed);
    double rotationSpeed = deltaRot * m_maxRotation;

    m_swerveSubsystem.drive(translationSpeeds, rotationSpeed, m_fieldRelative, m_openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gridFieldLineHelper.hide();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static class GridHelper {
    /** Grid values to control for */
    final double m_gridValues[];

    /** Max distance from a grid value to be within capture range */
    final double m_captureDistance;

    /**
     * Create an instance of the controller and set the grid values to snap to
     *
     * @param gridValues Grid values to snap to
     */
    public GridHelper(double gridValues[], double captureDistance) {
      m_gridValues = gridValues.clone();
      m_captureDistance = captureDistance;
    }

    /**
     * Returns the nearest grid value to snap to or null if no snap value is in capture range
     *
     * @param val Current value to find on the grid
     */
    private Double getNearestGridValue(double val) {
      Double result = null;
      for (double gridVal : m_gridValues) {
        double distance = gridVal - val;
        if (Math.abs(distance) <= m_captureDistance) {
          result = Double.valueOf(gridVal);
          break;
        }
      }

      return result;
    }
  }

  private static class GridFieldLinesHelper {
    private static final double kGridValues[] = SnapToGrid.kGridYValues;
    private static final double kFieldWidth = Units.inchesToMeters(315.5) * 2;
    private final HashMap<String, PathPlannerTrajectory> m_gridLineMap = new HashMap<>();
    private final Field2d m_field2d;

    public GridFieldLinesHelper(Field2d field) {
      m_field2d = field;
      double fieldWidth = kFieldWidth;
      PathConstraints constraints = new PathConstraints(0.5, 0.5);
      Rotation2d rot = new Rotation2d(0.0);
      for (double gridY : kGridValues) {
        PathPoint start = new PathPoint(new Translation2d(0.0, gridY), rot);
        PathPoint end = new PathPoint(new Translation2d(fieldWidth, gridY), rot);
        String name = String.format("GridY%.2f", gridY);
        PathPlannerTrajectory traj = PathPlanner.generatePath(constraints, start, end);
        m_gridLineMap.put(name, traj);
      }
    }

    public void display() {
      for (HashMap.Entry<String, PathPlannerTrajectory> set : m_gridLineMap.entrySet()) {
        m_field2d.getObject(set.getKey()).setTrajectory(set.getValue());
      }
    }

    public void hide() {
      PathPlannerTrajectory emptyTraj = new PathPlannerTrajectory();
      for (HashMap.Entry<String, PathPlannerTrajectory> set : m_gridLineMap.entrySet()) {
        m_field2d.getObject(set.getKey()).setTrajectory(emptyTraj);
      }
    }
  }
}
