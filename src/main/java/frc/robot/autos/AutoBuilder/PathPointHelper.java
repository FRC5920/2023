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
package frc.robot.autos.AutoBuilder;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A helper class that extends PathPoint to provide a printable waypoint */
public class PathPointHelper extends PathPoint {
  public final String name;

  /**
   * Creates a PathPointHelper object from given parameters
   *
   * @param nameStr Friendly name printed during debugging
   * @param x X coordinate
   * @param y Y coordinate
   * @param theta Trajectory heading at the point
   * @param holRot Holonomic rotation at the point
   */
  public PathPointHelper(String nameStr, double x, double y, Rotation2d holRot, Rotation2d theta) {
    super(new Translation2d(x, y), theta, holRot);
    name = nameStr;
  }

  /**
   * Creates a PathPointHelper object from given parameters
   *
   * @param nameStr Friendly name printed during debugging
   * @param pose X,Y position and holonomic rotation of the point
   * @param theta Trajectory heading at the point
   */
  public PathPointHelper(String nameStr, Pose2d pose, Rotation2d theta) {
    super(pose.getTranslation(), theta, pose.getRotation());
    name = nameStr;
  }

  /**
   * Creates a PathPointHelper object from given parameters
   *
   * @param nameStr Friendly name printed during debugging
   * @param pose X,Y position and holonomic rotation of the point
   * @param theta Trajectory heading at the point
   */
  public PathPointHelper(String nameStr, PathPoint point) {
    super(point.position, point.heading, point.holonomicRotation);
    name = nameStr;
  }

  public String format() {
    return String.format(
        "<%s> (x=%6f, y=%.6f), theta=%d deg, holonomic=%d deg",
        name,
        position.getX(),
        position.getY(),
        (int) heading.getDegrees(),
        (int) holonomicRotation.getDegrees());
  }

  public double getX() {
    return position.getX();
  }

  public double getY() {
    return position.getY();
  }

  public Translation2d getPosition() {
    return position;
  }

  public Rotation2d getHeading() {
    return heading;
  }

  public Rotation2d getHolonomicRotation() {
    return holonomicRotation;
  }

  public double distance(PathPointHelper other) {
    return position.getDistance(other.position);
  }

  public PathPointHelper clone(PathPointHelper other) {
    return new PathPointHelper(other.name, (PathPoint) other);
  }
}
