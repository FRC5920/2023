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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.thirdparty.FRC6328.FieldConstants;
import java.util.*;

public class AutoConstants {

  /** Physical attributes of the robot */
  public static class BotDimensions {

    /** Width of the square internal swerve drive base, bumpers not included */
    private static final double kDriveBaseWidthMeters = Units.inchesToMeters(26);
    /**
     * Distance from the drive base to the outside edge of the bumper on a given side of the robot
     */
    private static final double kBumperWidthMeters = Units.inchesToMeters(0.75 + 2.5);

    /** Total width of the (square) robot footprint in meters */
    private static final double kFootprintWidth = kDriveBaseWidthMeters + (2 * kBumperWidthMeters);

    /** Half the robot footprint width (i.e. the center of the robot) */
    private static final double kHalfFootprintWidth = kFootprintWidth / 2;
  }

  /** Constants indicating the robot's rotational orientation */
  public static class BotOrientation {
    /** Bot is rotated toward Grids */
    public static final Rotation2d kFacingGrid = new Rotation2d(Math.PI);
    /** Bot is rotated toward the middle of the field */
    public static final Rotation2d kFacingField = new Rotation2d(0.0);
    /** Bot is rotated facing in the opposite direction of the scoring table */
    public static final Rotation2d kFacingNorth = new Rotation2d(Math.PI / 2);
    /** Bot is rotated facing the scoring table */
    public static final Rotation2d kFacingSouth = new Rotation2d(1.5 * Math.PI);
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
      H(7),
      I(8);

      private static final Map<String, ScoringPosition> nameMap =
          Map.of("A", A, "B", B, "C", C, "D", D, "E", E, "F", F, "G", G, "H", H, "I", I);

      private final int id;
      public final Pose2d pose;

      private ScoringPosition(int idx) {
        id = idx;
        double x = kRobotCenterX;
        double y = kFirstGridCenterY + (kDistanceBetweenGridCenters * id);
        pose = new Pose2d(x, y, BotOrientation.kFacingGrid);
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
   * @note The OUTER lane is closest to the Grids, while the INNER lane is closest to the charging
   *     station
   */
  public static class Lanes {

    /** Margin introduced to lane centers */
    public static final double kLaneMargin = Units.inchesToMeters(4);

    /** Robot center X coordinate in the middle of the North-South section of the INNER lane */
    public static final double kInnerLaneColumnCenterX =
        ChargingStation.kGridSideX - (BotDimensions.kHalfFootprintWidth) - kLaneMargin;

    /** Robot center X coordinate in the middle of the North-South section of the OUTER lane */
    public static final double kOuterLaneColumnCenterX =
        Grids.kGridEdgeX + BotDimensions.kHalfFootprintWidth + kLaneMargin;

    /**
     * Y coordinate of the center of the East-West portion of the OUTER lane furthest from the
     * scoring table
     *
     * <p>TODO: this value needs experimental refinement!
     */
    public static double kOuterLaneNorthCenterY = Grids.ScoringPosition.I.pose.getY() - kLaneMargin;

    /**
     * Y coordinate in the center of the East-West portion of the INNER lane furthest from the
     * scoring table
     */
    public static double kInnerLaneNorthCenterY =
        ChargingStation.kNorthSideY + BotDimensions.kHalfFootprintWidth + kLaneMargin;

    /**
     * Y coordinate in the center of the East-West portion of the OUTER lane closest to the scoring
     * table
     */
    public static double kOuterLaneSouthCenterY =
        0.0 + BotDimensions.kHalfFootprintWidth + kLaneMargin;

    /**
     * Y coordinate in the center of the East-West portion of the INNER lane closest to the scoring
     * table
     */
    public static double kInnerLaneSouthCenterY =
        ChargingStation.kSouthSideY - BotDimensions.kHalfFootprintWidth - kLaneMargin;

    /**
     * Lanes
     *
     * @remarks Lanes are used to indicate imaginary lantes that run between the Grids and the
     *     Charging Station.
     */
    public static enum ID {
      Outer(0),
      Inner(1);

      private static final Map<String, ID> nameMap = Map.of("Inner", Inner, "Outer", Outer);

      private final int id;
      public final double xColumnCenter; // X-coordinate of the lane's North-South section

      private ID(int idx) {
        id = idx;
        xColumnCenter = (id == 1) ? kInnerLaneColumnCenterX : kOuterLaneColumnCenterX;
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

      public double getYForRoute(EscapeRoute.ID route) {
        double y = -1.0;
        switch (route) {
          case NorthOfCS: // North of CS
            y = (id == Inner.id) ? kInnerLaneNorthCenterY : kOuterLaneNorthCenterY;
            break;
          case SouthOfCS: // South of CS
            y = (id == Outer.id) ? kInnerLaneSouthCenterY : kOuterLaneSouthCenterY;
            break;
          case CenterOfCS: // Through CS
            y = ChargingStation.kCenterY;
            break;
        }
        return y;
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used to identify the route the bot will take out of the community */
  public static class EscapeRoute {
    public static enum ID {
      /**
       * This route takes the bot to the North of the charging station, on the side furthest from
       * the scoring table
       */
      NorthOfCS(0),
      /**
       * This route takes the bot to the South of the charging station, on the side closest to the
       * scoring table
       */
      SouthOfCS(1),
      /** This route takes the bot through the center of the Charging Station */
      CenterOfCS(2);

      private static final Map<String, ID> nameMap =
          Map.of("NorthOfCS", NorthOfCS, "SouthOfCS", SouthOfCS, "CenterOfCS", CenterOfCS);

      private final int id;

      private ID(int idx) {
        id = idx;
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
    public static enum Transitional {
      // Outer lane North corner waypoints
      OuterNorthCornerBottom(Lanes.kOuterLaneColumnCenterX, Lanes.kOuterLaneNorthCenterY - BotDimensions.kHalfFootprintWidth),
      OuterNorthCornerTop(Lanes.kOuterLaneColumnCenterX + BotDimensions.kHalfFootprintWidth, Lanes.kOuterLaneNorthCenterY),

      // Inner lane North corner waypoints
      InnerNorthCornerBottom(Lanes.kInnerLaneColumnCenterX, Lanes.kInnerLaneNorthCenterY - BotDimensions.kHalfFootprintWidth),
      InnerNorthCornerTop(Lanes.kInnerLaneColumnCenterX + BotDimensions.kHalfFootprintWidth, Lanes.kInnerLaneNorthCenterY),

      // Outer lane South corner waypoints
      OuterSouthCornerBottom(Lanes.kOuterLaneColumnCenterX, Lanes.kOuterLaneSouthCenterY - BotDimensions.kHalfFootprintWidth),
      OuterSouthCornerTop(Lanes.kOuterLaneColumnCenterX + BotDimensions.kHalfFootprintWidth, Lanes.kOuterLaneSouthCenterY),

      // Inner lane South corner waypoints
      InnerSouthCornerBottom(Lanes.kInnerLaneColumnCenterX, Lanes.kInnerLaneSouthCenterY - BotDimensions.kHalfFootprintWidth),
      InnerSouthCornerTop(Lanes.kInnerLaneColumnCenterX + BotDimensions.kHalfFootprintWidth, Lanes.kInnerLaneSouthCenterY),

      /**
       * OUTER lane North of the charging station aligned with Fieldward side of Charging Station
       */
      OuterLaneNorthOfCS(ChargingStation.kFieldSideX, Lanes.kOuterLaneNorthCenterY),
      /**
       * INNER lane North of the charging station aligned with Fieldward side of Charging Station
       */
      InnerLaneNorthOfCS(ChargingStation.kFieldSideX, Lanes.kInnerLaneNorthCenterY),

      /**
       * OUTER lane South of the charging station aligned with Fieldward side of Charging Station
       */
      OuterLaneSouthOfCS(ChargingStation.kFieldSideX + 0.1, Lanes.kOuterLaneSouthCenterY),
      /**
       * INNER lane South of the charging station aligned with Fieldward side of Charging Station
       */
      InnerLaneSouthOfCS(ChargingStation.kFieldSideX + 0.1, Lanes.kInnerLaneSouthCenterY);

      public final Translation2d coordinates;

      private Transitional(double x, double y) {
        coordinates = new Translation2d(x, y);
      }
    }

    private static final List<Transitional> kOuterNorthCornerList = List.of(Transitional.OuterNorthCornerBottom, Transitional.OuterNorthCornerTop);
    private static final List<Transitional> kOuterSouthCornerList = List.of(Transitional.OuterSouthCornerBottom, Transitional.OuterSouthCornerTop);

    private static final List<Transitional> kInnerNorthCornerList = List.of(Transitional.InnerNorthCornerTop, Transitional.InnerNorthCornerBottom);
    private static final List<Transitional> kInnerSouthCornerList = List.of(Transitional.InnerSouthCornerTop, Transitional.InnerSouthCornerBottom);

    public static final Map<String, List<Transitional>> cornerMap = Map.of(
      (Lanes.ID.Outer.name() + EscapeRoute.ID.NorthOfCS.name()), kOuterNorthCornerList,
      (Lanes.ID.Inner.name() + EscapeRoute.ID.NorthOfCS.name()), kInnerNorthCornerList,
      (Lanes.ID.Outer.name() + EscapeRoute.ID.SouthOfCS.name()), kOuterSouthCornerList,
      (Lanes.ID.Inner.name() + EscapeRoute.ID.SouthOfCS.name()), kInnerSouthCornerList
    );

    public static final Map<String, Transitional> transitionalMap = Map.of(
      (Lanes.ID.Outer.name() + EscapeRoute.ID.NorthOfCS.name()), Transitional.OuterLaneNorthOfCS,
      (Lanes.ID.Inner.name() + EscapeRoute.ID.NorthOfCS.name()), Transitional.InnerLaneNorthOfCS,
      (Lanes.ID.Outer.name() + EscapeRoute.ID.SouthOfCS.name()), Transitional.OuterLaneSouthOfCS,
      (Lanes.ID.Inner.name() + EscapeRoute.ID.SouthOfCS.name()), Transitional.InnerLaneSouthOfCS
    );
    
    public static enum ID {
      // Far North staging areas next to center line
      U(2, 7.5, 7.5), // Furthest north next to center line
      V(3, 7.5, 6.0), // Secondmost north next to center line

      // Staging areas between charging station and cargo
      X(4, 6.0, ChargingStation.kNorthSideY), // North of charging station
      Y(5, 6.0, ChargingStation.kCenterY), // Centered on charging station
      Z(6, 6.0, ChargingStation.kSouthSideY); // South of charging station

      private static final Map<String, ID> nameMap =
          Map.of(
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

      /** Coordinates on the field */
      public final Translation2d coordinates;

      private ID(int idx, double x, double y) {
        id = idx;
        coordinates = new Translation2d(x, y);
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

    /** Coordinates on the field */
    public final Translation2d coordinates;

    private CargoLocation(int idx) {
      id = idx;
      double x = FieldConstants.StagingLocations.translations[id].getX();
      double y = FieldConstants.StagingLocations.translations[id].getY();
      coordinates = new Translation2d(x, y);
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
