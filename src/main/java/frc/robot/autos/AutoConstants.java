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
package frc.robot.autos;

import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import java.util.*;

public class AutoConstants {

  /** Physical attributes of the robot */
  public static class BotDimensions {

    /** Width of the square internal swerve drive base, bumpers not included */
    private static double kDriveBaseWidthMeters = Units.inchesToMeters(26);
    /**
     * Distance from the drive base to the outside edge of the bumper on a given side of the robot
     */
    private static double kBumperWidthMeters = Units.inchesToMeters(3.25);

    /** Total width of the (square) robot footprint in meters */
    public static double kFootprintWidth = kDriveBaseWidthMeters + (2 * kBumperWidthMeters);

    /** Half the robot footprint width (i.e. the center of the robot) */
    public static double kHalfFootprintWidth = kFootprintWidth / 2;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used in conjunction with Grids where cargo is placed */
  public static class Grids {
    /** X coordinate where the grids meet the field */
    static final double kGridEdgeX = FieldConstants.Grids.outerX;

    /** X coordinate of the robot center when the robot is in scoring position at a Grid */
    static final double kRobotCenterX = (kGridEdgeX + BotDimensions.kHalfFootprintWidth);

    /** Y coordinate of the center of the first Grid (closest to scoring tables) */
    static final double kFirstGridCenterY = FieldConstants.Grids.nodeFirstY;

    /** Distance between centerlines of adjacent Grids */
    static final double kDistanceBetweenGridCenters = FieldConstants.Grids.nodeSeparationY;

    /**
     * Individual Grid positions
     *
     * @remarks Individual Grid positions are enumerated using letters A-I, with A being closest to
     *     the scoring table.
     */
    public static enum ScoringPosition {
      A(0),
      B(1),
      C(2),
      D(3),
      E(4),
      F(5),
      G(6),
      H(7);

      private static final Map<String, ScoringPosition> nameMap =
          Map.of("A", A, "B", B, "C", C, "D", D, "E", E, "F", F, "G", G, "H", H);

      private final int id;
      private final double x; // X coordinate
      private final double y; // Y coordinate

      private ScoringPosition(int idx) {
        id = idx;
        x = kRobotCenterX;
        y = kFirstGridCenterY + (kDistanceBetweenGridCenters * id);
      }

      /** Get the human-readable name of the robot type */
      @Override
      public String toString() {
        return this.name();
      }

      public static ScoringPosition fromName(String s) {
        return nameMap.get(s);
      }

      public static String[] getNames() {
        String names[] = nameMap.keySet().toArray(new String[nameMap.size()]);
        Arrays.sort(names);
        return names;
      }
    };
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used in conjunction with the Charging Station */
  public static class ChargingStation {
    /** X coordinate of the side of the charging station closest to the Grids */
    public static final double kGridSideX = FieldConstants.Community.chargingStationInnerX;
    /** X coordinate of the side of the charging station furthest from the Grids */
    public static final double kFieldSideX = FieldConstants.Community.chargingStationOuterX;

    /** Y coordinate of the side of the charging station closest to the scoring table */
    public static final double kSouthSideY = FieldConstants.Community.chargingStationLeftY;
    /** Y coordinate of the side of the charging station furthest from the Grids */
    public static final double kNorthSideY = FieldConstants.Community.chargingStationRightY;

    /** X coordinate of the center of the Charging Station */
    public static final double kCenterX = (kFieldSideX - kGridSideX) / 2.0;
    /** Y coordinate of the center of the Charging Station */
    public static final double kCenterY = (kNorthSideY - kSouthSideY) / 2.0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Constants used in conjunction with imaginary "lanes" that run between the Grids and the
   * Charging Station.
   *
   * @note The INSIDE lane is closest to the Grids, while the OUTSIDE lane is closest to the
   *     charging station
   */
  public static class Lanes {

    /** Robot center X coordinate in the middle of the North-South section of the INSIDE lane */
    public static double kNSInsideCenterX = Grids.kRobotCenterX;

    /** Robot center X coordinate in the middle of the North-South section of the OUTSIDE lane */
    public static double kNSOutsideCenterX =
        ChargingStation.kGridSideX - (BotDimensions.kHalfFootprintWidth);

    /**
     * Robot center Y coordinate in the East-West portion of the INSIDE lane furthest from the
     * scoring table
     */
    public static double kUpperEastWestInsideCenterY =
        FieldConstants.Community.midY - BotDimensions.kHalfFootprintWidth;

    /**
     * Robot center Y coordinate in the East-West portion of the OUTSIDE lane furthest from the
     * scoring table
     */
    public static double kUpperEastWestOutsideCenterY =
        ChargingStation.kNorthSideY + BotDimensions.kHalfFootprintWidth;

    /**
     * Robot center Y coordinate in the East-West portion of the INSIDE lane closest to the scoring
     * table
     */
    public static double kLowerEastWestInsideCenterY = 0.0 + BotDimensions.kHalfFootprintWidth;

    /**
     * Robot center Y coordinate in the East-West portion of the OUTSIDE lane closest to the scoring
     * table
     */
    public static double kLowerEastWestOutsideCenterY =
        ChargingStation.kSouthSideY - BotDimensions.kHalfFootprintWidth;

    /**
     * Lanes
     *
     * @remarks Staging points are used to indicate transitional points where the robot might be
     *     positioned in an auto routine
     */
    public static enum ID {
      Inside(0),
      Outside(1);

      private static final Map<String, ID> nameMap = Map.of("Inside", Inside, "Outside", Outside);

      private final int id;
      private final double xNorthSouth; // X-coordinate of the lane's North-South section
      private final double yUpperEastWest; // Y-coordinate of the lane's upper East-West section
      private final double yLowerEastWest; // Y-coordinate of the lane's upper East-West section

      private ID(int idx) {
        id = idx;
        if (id == 0) {
          xNorthSouth = kNSInsideCenterX;
          yUpperEastWest = kUpperEastWestInsideCenterY;
          yLowerEastWest = kLowerEastWestInsideCenterY;
        } else {
          xNorthSouth = kNSOutsideCenterX;
          yUpperEastWest = kUpperEastWestOutsideCenterY;
          yLowerEastWest = kLowerEastWestOutsideCenterY;
        }
      }

      /** Get the human-readable name of the robot type */
      @Override
      public String toString() {
        return this.name();
      }

      public static ID fromString(String s) {
        return nameMap.get(s);
      }

      public static String[] getNames() {
        String names[] = nameMap.keySet().toArray(new String[nameMap.size()]);
        Arrays.sort(names);
        return names;
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used in conjunction with predefined waypoints used in auto routines */
  public static class Waypoints {
    public static enum ID {
      /** Inside lane beyond tape just North of the charging station */
      InsideNorth(0, 4.0, Lanes.kUpperEastWestInsideCenterY),
      /** Inside lane beyond tape South of the charging station */
      InsideSouth(0, ChargingStation.kFieldSideX + 0.1, Lanes.kLowerEastWestInsideCenterY),

      /** Outside lane beyond tape just North of the charging station */
      OutsideNorth(1, 4.0, Lanes.kUpperEastWestOutsideCenterY),
      /** Outside lane beyond tape South of the charging station */
      OutsideSouth(0, ChargingStation.kFieldSideX + 0.1, Lanes.kLowerEastWestOutsideCenterY),

      // Far North staging areas next to center line
      U(2, 7.5, 7.5), // Furthest north next to center line
      V(3, 7.5, 6.0), // Secondmost north next to center line

      // Staging areas between charging station and cargo
      X(4, 6.0, ChargingStation.kNorthSideY), // North of charging station
      Y(5, 6.0, ChargingStation.kCenterY), // Centered on charging station
      Z(6, 6.0, ChargingStation.kSouthSideY); // South of charging station

      private static final Map<String, ID> nameMap =
          Map.of(
              "InsideNorth",
              InsideNorth,
              "InsideSouth",
              InsideSouth,
              "OutsideNorth",
              OutsideNorth,
              "OutsideSouth",
              OutsideSouth,
              "U",
              U,
              "V",
              V,
              "X",
              X,
              "Y",
              Y,
              "Z",
              Z);

      private final int id;
      private final double x; // X coordinate
      private final double y; // Y coordinate

      private ID(int idx, double xLoc, double yLoc) {
        id = idx;
        x = xLoc;
        y = yLoc;
      }

      /** Get the human-readable name of the robot type */
      @Override
      public String toString() {
        return this.name();
      }

      public static ID fromString(String s) {
        return nameMap.get(s);
      }

      public static String[] getNames() {
        String names[] = nameMap.keySet().toArray(new String[nameMap.size()]);
        Arrays.sort(names);
        return names;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Staging points
   *
   * @remarks Staging points are used to indicate transitional points where the robot might be
   *     positioned in an auto routine
   */
  public static enum CargoLocation {
    C1(0), // North of the charging station
    C2(1), // Centered on northern half of the charging station
    C3(2), // Centered on southern half of the charging station
    C4(3); // South of the charging station

    private static final Map<String, CargoLocation> nameMap =
        Map.of("C1", C1, "C2", C2, "C3", C3, "C4", C4);

    private final int id;
    private final double x; // X coordinate
    private final double y; // Y coordinate

    private CargoLocation(int idx) {
      id = idx;
      x = FieldConstants.StagingLocations.translations[id].getX();
      y = FieldConstants.StagingLocations.translations[id].getY();
    }

    /** Get the human-readable name of the robot type */
    @Override
    public String toString() {
      return this.name();
    }

    public static CargoLocation fromString(String s) {
      return nameMap.get(s);
    }

    public static String[] getNames() {
      String names[] = nameMap.keySet().toArray(new String[nameMap.size()]);
      Arrays.sort(names);
      return names;
    }
  };

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Staging points
   *
   * @remarks Staging points are used to indicate transitional points where the robot might be
   *     positioned in an auto routine
   */
  public static enum SecondaryAction {
    WaitAtLocation(0), // Go to a location and wait
    Balance(1), // Balance on the charging station
    AcquireCargo(2); // Acquire a piece of cargo

    private static final Map<String, SecondaryAction> nameMap =
        Map.of("WaitAtLocation", WaitAtLocation, "Balance", Balance, "AcquireCargo", AcquireCargo);

    private final int id;

    private SecondaryAction(int idx) {
      id = idx;
    }

    /** Get the human-readable name of the robot type */
    @Override
    public String toString() {
      return this.name();
    }

    public static SecondaryAction fromString(String s) {
      return nameMap.get(s);
    }

    public static String[] getNames() {
      String names[] = nameMap.keySet().toArray(new String[nameMap.size()]);
      Arrays.sort(names);
      return names;
    }
  };
}
