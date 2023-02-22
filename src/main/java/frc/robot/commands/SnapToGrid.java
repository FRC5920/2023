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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Joystick.ProcessedXboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;

public class SnapToGrid extends CommandBase {
  /** Creates a new SnapToGrid. */
  private double rotation;

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  private boolean foundSnapPoint[] = new boolean[2];
  int timesGotXFromJoystick;
  double xAxis;

  private static final double GridkP = 0.5;
  private static final double GridkI = 0.05;
  private static final double GridkD = 0.2;
  PIDController gridPID = new PIDController(GridkP, GridkI, GridkD);

  /** Y coordinates of grid lines that line up with goal stations */
  final GridLine yGridLines[];

  private Swerve s_Swerve;
  private ProcessedXboxController controller;

  public SnapToGrid(
      Swerve s_Swerve,
      JoystickSubsystem joystickSubsystem,
      boolean fieldRelative,
      boolean openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    foundSnapPoint[0] = foundSnapPoint[1] = false;

    // Create y grid values
    yGridLines = new GridLine[FieldConstants.Grids.nodeRowCount];
    for (int i = 0; i < FieldConstants.Grids.nodeRowCount; i++) {
      yGridLines[i] = new GridLine(FieldConstants.Grids.lowTranslations[i].getY());
    }

    this.controller = joystickSubsystem.driverController;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timesGotXFromJoystick = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxis = -controller.getLeftX();
    double yAxis = -controller.getLeftY();
    double rAxis = -controller.getRightX();

    // Get the current Y coordinate
    Pose2d currentPose = s_Swerve.getPose();
    double currentY = currentPose.getY();
    SmartDashboard.putNumber("currentX", currentPose.getX());
    SmartDashboard.putNumber("currentY", currentPose.getY());

    // Rotate history
    foundSnapPoint[1] = foundSnapPoint[0];
    foundSnapPoint[0] = false;
    double distanceToGrid = 0;

    // Check if our current Y coordinate is within capture range of a grid line
    for (GridLine gridLine : yGridLines) {
      if (gridLine.isInCaptureRange(currentY)) {
        foundSnapPoint[0] = true;
        distanceToGrid = gridLine.distanceFromGridLine(currentY);

        if (foundSnapPoint[0] && !foundSnapPoint[1]) {
          gridPID.reset();
        }
        gridPID.setSetpoint(gridLine.gridY);
        break;
      }
    }

    SmartDashboard.putBoolean("foundSnapPoint", foundSnapPoint[0]);
    if (foundSnapPoint[0]) {
      SmartDashboard.putNumber("Set point Y", gridPID.getSetpoint());
      SmartDashboard.putNumber("distanceToGrid", distanceToGrid);
    }
    double xSpeed = yAxis;
    double ySpeed = (foundSnapPoint[0]) ? gridPID.calculate(currentY) : xAxis;

    translation = new Translation2d(xSpeed, ySpeed).times(BotStateSubsystem.MaxSpeed);
    SmartDashboard.putNumber("xSpeed", translation.getX());
    SmartDashboard.putNumber("ySpeed", translation.getY());
    rotation = rAxis * BotStateSubsystem.MaxRotate;

    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Grid lines to snap to */
  private class GridLine {
    private final double kCaptureRangeMeters = Units.inchesToMeters(11);
    private final double gridY;

    /** Creates a GridLine with a specified Y coordinate */
    public GridLine(double y) {
      gridY = y;
    }

    /**
     * Returns true if a given Y coordinate is within the capture range of the GridLine
     *
     * @param y Y coordinate to test
     */
    public boolean isInCaptureRange(double y) {
      return (Math.abs(distanceFromGridLine(y)) <= kCaptureRangeMeters);
    }

    /** Returns the distance (meters) between the GridLine and a given Y coordinate */
    public double distanceFromGridLine(double y) {
      return gridY - y;
    }
  }
}
