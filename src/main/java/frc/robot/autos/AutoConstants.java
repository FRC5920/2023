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
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
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
    public static final double kFootprintWidth = kDriveBaseWidthMeters + (2 * kBumperWidthMeters);

    /** Half the robot footprint width (i.e. the center of the robot) */
    public static final double kHalfFootprintWidth = kFootprintWidth / 2;
  }

  /** Constants indicating the robot's rotational orientation */
  public static class BotOrientation {
    /** Bot is rotated toward Grids */
    private static final Rotation2d kFacingGrid = new Rotation2d(Math.PI);
    /** Bot is rotated toward the middle of the field */
    private static final Rotation2d kFacingField = new Rotation2d(0.0);
    /** Bot is rotated facing in the opposite direction of the scoring table */
    public static final Rotation2d kFacingNorth = new Rotation2d(Math.PI / 2.0);
    /** Bot is rotated facing the scoring table */
    public static final Rotation2d kFacingSouth = new Rotation2d(1.5 * Math.PI);

    /** Returns a Rotation2d pointed toward the current Alliance's Grid */
    public static Rotation2d facingGrid() {
      return new Rotation2d(Math.PI);
    }

    /** Returns a Rotation2d pointed toward the center of the field */
    public static Rotation2d facingField() {
      return new Rotation2d(0.0);
    }
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

      private final int id;
      private final Translation2d position;

      private ScoringPosition(int idx) {
        id = idx;
        double x = kRobotCenterX;
        double y = kFirstGridCenterY + (kDistanceBetweenGridCenters * id);
        position = new Translation2d(x, y);
      }

      public Pose2d getPose() {
        return AllianceFlipUtil.apply(new Pose2d(position, BotOrientation.facingGrid()));
      }

      public Translation2d getPosition() {
        return AllianceFlipUtil.apply(position);
      }

      public static String[] getNames() {
        return getEnumNames(ScoringPosition.class);
      }
    };
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used in conjunction with the Charging Station */
  public static class ChargingStation {
    /** X coordinate of the side of the charging station closest to the Grids */
    private static final double kGridSideX = FieldConstants.Community.chargingStationInnerX;
    /** X coordinate of the side of the charging station furthest from the Grids */
    private static final double kFieldSideX = FieldConstants.Community.chargingStationOuterX;

    /** Y coordinate of the side of the charging station closest to the scoring table */
    private static final double kSouthSideY = FieldConstants.Community.chargingStationRightY;
    /** Y coordinate of the side of the charging station furthest from the Grids */
    private static final double kNorthSideY = FieldConstants.Community.chargingStationLeftY;

    /** X coordinate of the center of the Charging Station */
    private static final double kCenterX =
        (kFieldSideX - (FieldConstants.Community.chargingStationLength / 2));
    /** Y coordinate of the center of the Charging Station */
    private static final double kCenterY =
        (kSouthSideY + (FieldConstants.Community.chargingStationWidth / 2));

    public static Translation2d kGridSideNorth = new Translation2d(kGridSideX, kNorthSideY);
    public static Translation2d kFieldSideSouth = new Translation2d(kGridSideX, kNorthSideY);
    public static Translation2d kCenter = new Translation2d(kCenterX, kCenterY);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used to identify the route the bot will take out of the community */
  public static class EscapeRoute {

    /** X coordinate of escape route endpoint */
    public static final double kEscapeEndpointX =
        ChargingStation.kFieldSideX + (0.75 * BotDimensions.kFootprintWidth);

    public static enum Route {
      /**
       * This route takes the bot to the North of the charging station (the side furthest from the
       * scoring table) via the OUTER lane
       */
      NorthOfCSViaOuterLane,
      /**
       * This route takes the bot to the North of the charging station (the side furthest from the
       * scoring table) via the INNER lane
       */
      NorthOfCSViaInnerLane,
      /**
       * This route takes the bot to the South of the charging station (the side closest to the
       * scoring table) via the OUTER lane
       */
      SouthOfCSViaOuterLane,
      /**
       * This route takes the bot to the South of the charging station (the side closest to the
       * scoring table) via the OUTER lane
       */
      SouthOfCSViaInnerLane,
      /** This route takes the bot through the center of the Charging Station via the OUTER lane */
      ThroughCSViaOuterLane,
      /** This route takes the bot through the center of the Charging Station via the INNER lane */
      ThroughCSViaInnerLane;

      /** Returns a list of names of enum elements */
      public static String[] getNames() {
        return getEnumNames(Route.class);
      }
    }

    /**
     * Lanes
     *
     * @remarks Lanes are used to indicate imaginary lantes that run between the Grids and the
     *     Charging Station.
     */
    public static enum Lane {
      Outer(0),
      Inner(1);

      private final int id;
      public final double xColumnCenter; // X-coordinate of the lane's North-South section

      private Lane(int idx) {
        id = idx;
        xColumnCenter =
            (id == 1)
                ? LaneConstants.kInnerLaneColumnCenterX
                : LaneConstants.kOuterLaneColumnCenterX;
      }

      /** Returns a list of names of enum elements */
      public static String[] getNames() {
        return getEnumNames(Lane.class);
      }
    }

    public static enum Corner {
      // Outer lane North corner waypoints
      OuterLaneNorthCorner(
          LaneConstants.kOuterLaneColumnCenterX, LaneConstants.kOuterLaneNorthCenterY),
      OuterLaneSouthCorner(
          LaneConstants.kOuterLaneColumnCenterX, LaneConstants.kOuterLaneSouthCenterY),
      OuterLaneCenterOfCS(LaneConstants.kOuterLaneColumnCenterX, ChargingStation.kCenterY),

      InnerLaneNorthCorner(
          LaneConstants.kInnerLaneColumnCenterX, LaneConstants.kInnerLaneNorthCenterY),
      InnerLaneSouthCorner(
          LaneConstants.kInnerLaneColumnCenterX, LaneConstants.kInnerLaneSouthCenterY),
      InnerLaneCenterOfCS(LaneConstants.kInnerLaneColumnCenterX, ChargingStation.kCenterY);

      private final Translation2d position;

      private Corner(double x, double y) {
        position = new Translation2d(x, y);
      }

      public Translation2d getPosition() {
        return AllianceFlipUtil.apply(position);
      }
    }

    public static enum Endpoint {
      /** End of the North OUTER lane aligned with Fieldward side of Charging Station */
      OuterLaneNorthEndpoint(kEscapeEndpointX, LaneConstants.kOuterLaneNorthCenterY),
      /** End of the North INNER lane aligned with Fieldward side of Charging Station */
      InnerLaneNorthEndpoint(kEscapeEndpointX, LaneConstants.kInnerLaneNorthCenterY),

      /** End of the South OUTER lane just past tape South of Charging Station */
      OuterLaneSouthEndpoint(kEscapeEndpointX, LaneConstants.kOuterLaneSouthCenterY),
      /** End of the South INNER lane just past tape South of Charging Station */
      InnerLaneSouthEndpoint(kEscapeEndpointX, LaneConstants.kInnerLaneSouthCenterY),

      /** End of the route through the Charging Station via the Outer lane */
      ThroughCSEndpoint(kEscapeEndpointX, ChargingStation.kCenterY);

      private final Translation2d position;

      private Endpoint(double x, double y) {
        position = new Translation2d(x, y);
      }

      public Translation2d getPosition() {
        return AllianceFlipUtil.apply(position);
      }
    }

    /** Returns the lane corresponding to a given route */
    public static Lane getLane(Route route) {
      return laneMap.get(route);
    }

    /** Returns the corner associated with a given route */
    public static Corner getCorner(Route route) {
      return cornerMap.get(route);
    }

    /** Returns the endpoint associated with a given route */
    public static Endpoint getEndpoint(Route route) {
      return endpointMap.get(route);
    }

    private static final Map<Route, Lane> laneMap =
        Map.of(
            Route.NorthOfCSViaOuterLane, Lane.Outer,
            Route.NorthOfCSViaInnerLane, Lane.Inner,
            Route.SouthOfCSViaOuterLane, Lane.Outer,
            Route.SouthOfCSViaInnerLane, Lane.Inner,
            Route.ThroughCSViaOuterLane, Lane.Outer,
            Route.ThroughCSViaInnerLane, Lane.Inner);

    private static final Map<Route, Corner> cornerMap =
        Map.of(
            Route.NorthOfCSViaOuterLane, Corner.OuterLaneNorthCorner,
            Route.NorthOfCSViaInnerLane, Corner.InnerLaneNorthCorner,
            Route.SouthOfCSViaOuterLane, Corner.OuterLaneSouthCorner,
            Route.SouthOfCSViaInnerLane, Corner.InnerLaneSouthCorner,
            Route.ThroughCSViaOuterLane, Corner.OuterLaneCenterOfCS,
            Route.ThroughCSViaInnerLane, Corner.InnerLaneCenterOfCS);

    private static final Map<Route, Endpoint> endpointMap =
        Map.of(
            Route.NorthOfCSViaOuterLane, Endpoint.OuterLaneNorthEndpoint,
            Route.NorthOfCSViaInnerLane, Endpoint.InnerLaneNorthEndpoint,
            Route.SouthOfCSViaOuterLane, Endpoint.OuterLaneSouthEndpoint,
            Route.SouthOfCSViaInnerLane, Endpoint.InnerLaneSouthEndpoint,
            Route.ThroughCSViaOuterLane, Endpoint.ThroughCSEndpoint,
            Route.ThroughCSViaInnerLane, Endpoint.ThroughCSEndpoint);

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Constants used in conjunction with imaginary "lanes" that run between the Grids and the
     * Charging Station.
     *
     * @note The OUTER lane is closest to the Grids, while the INNER lane is closest to the charging
     *     station
     */
    private static class LaneConstants {

      /** Margin introduced to lane centers */
      public static final double kLaneMargin = Units.inchesToMeters(4);

      /** Robot center X coordinate in the middle of the North-South section of the INNER lane */
      public static final double kInnerLaneColumnCenterX =
          ChargingStation.kGridSideX - BotDimensions.kHalfFootprintWidth - kLaneMargin;

      /** Robot center X coordinate in the middle of the North-South section of the OUTER lane */
      public static final double kOuterLaneColumnCenterX =
          Grids.kGridEdgeX + BotDimensions.kHalfFootprintWidth + kLaneMargin;

      /**
       * Y coordinate of the center of the East-West portion of the OUTER lane furthest from the
       * scoring table
       *
       * <p>TODO: this value needs experimental refinement!
       */
      public static double kOuterLaneNorthCenterY =
          Grids.ScoringPosition.I.position.getY() - kLaneMargin;

      /**
       * Y coordinate in the center of the East-West portion of the INNER lane furthest from the
       * scoring table
       */
      public static double kInnerLaneNorthCenterY =
          ChargingStation.kNorthSideY + BotDimensions.kHalfFootprintWidth + kLaneMargin;

      /**
       * Y coordinate in the center of the East-West portion of the OUTER lane closest to the
       * scoring table
       */
      public static double kOuterLaneSouthCenterY =
          0.0 + BotDimensions.kHalfFootprintWidth + kLaneMargin;

      /**
       * Y coordinate in the center of the East-West portion of the INNER lane closest to the
       * scoring table
       */
      public static double kInnerLaneSouthCenterY =
          ChargingStation.kSouthSideY - BotDimensions.kHalfFootprintWidth - kLaneMargin;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Constants used in conjunction with predefined waypoints used in auto routines */
  public static class Waypoints {

    public static enum ID {
      // Far North staging areas next to center line
      U(7.5, 7.5), // Furthest north next to center line
      V(7.5, 6.0), // Secondmost north next to center line

      // Staging areas between charging station and cargo
      X(6.0, ChargingStation.kNorthSideY), // North of charging station
      Y(6.0, ChargingStation.kCenterY), // Centered on charging station
      Z(6.0, ChargingStation.kSouthSideY); // South of charging station

      /** Coordinates on the field */
      public final Translation2d coordinates;

      private ID(double x, double y) {
        coordinates = new Translation2d(x, y);
      }

      /** Returns a list of names of enum elements */
      public static String[] getNames() {
        return getEnumNames(ID.class);
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

    /** Coordinates on the field */
    public final Translation2d coordinates;

    private CargoLocation(int id) {
      double x = FieldConstants.StagingLocations.translations[id].getX();
      double y = FieldConstants.StagingLocations.translations[id].getY();
      coordinates = new Translation2d(x, y);
    }

    /** Returns a list of names of enum elements */
    public static String[] getNames() {
      return getEnumNames(CargoLocation.class);
    }
  };

  /////////////////////////////////////////////////////////////////////////////
  /** An enumeration of secondary actions to take after escaping the community */
  public static enum SecondaryAction {
    WaitAtLocation, // Go to a location and wait
    Balance, // Balance on the charging station
    AcquireCargo; // Acquire a piece of cargo

    /** Returns a list of names of enum elements */
    public static String[] getNames() {
      return getEnumNames(SecondaryAction.class);
    }
  };

  private static String[] getEnumNames(Class<? extends Enum<?>> e) {
    return Arrays.stream(e.getEnumConstants()).map(Enum::name).toArray(String[]::new);
  }
}
