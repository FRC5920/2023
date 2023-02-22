package frc.robot;

import java.util.*;

public class AutoConstants {
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Goal stations
   *
   * @remarks Goal stations are named from A-I, always starting with the station closest to the
   *     adjoining alliance's feeder station.
   */
  public enum GoalLocation {
    // TODO: add coordinates
    A(0, 0.0, 0.0),
    B(1, 0.0, 0.0),
    C(2, 0.0, 0.0),
    D(3, 0.0, 0.0),
    E(4, 0.0, 0.0),
    F(5, 0.0, 0.0),
    G(6, 0.0, 0.0),
    H(7, 0.0, 0.0);

    private static final Map<String, GoalLocation> nameMap =
        Map.of("A", A, "B", B, "C", C, "D", D, "E", E, "F", F, "G", G, "H", H);

    private final int id;
    private final double x; // X coordinate
    private final double y; // Y coordinate

    private GoalLocation(int idx, double xLoc, double yLoc) {
      id = idx;
      x = xLoc;
      y = yLoc;
    }

    /** Get the human-readable name of the robot type */
    @Override
    public String toString() {
      return this.name();
    }

    public static GoalLocation fromString(String s) {
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
  public enum StagingLocation {
    // TODO: add coordinates
    N(0, 0.0, 0.0), // Just North of the charging station
    S(1, 0.0, 0.0), // Just South of the charging station

    // Far North staging areas next to center line
    U(2, 0.0, 0.0), // Furthest north next to center line
    V(2, 0.0, 0.0), // Secondmost north next to center line

    // Staging areas between charging station and cargo
    X(2, 0.0, 0.0), // North of charging station
    Y(2, 0.0, 0.0), // Centered on charging station
    Z(2, 0.0, 0.0); // South of charging station

    private static final Map<String, StagingLocation> nameMap =
        Map.of("N", N, "S", S, "U", U, "V", V, "X", X, "Y", Y, "Z", Z);

    private final int id;
    private final double x; // X coordinate
    private final double y; // Y coordinate

    private StagingLocation(int idx, double xLoc, double yLoc) {
      id = idx;
      x = xLoc;
      y = yLoc;
    }

    /** Get the human-readable name of the robot type */
    @Override
    public String toString() {
      return this.name();
    }

    public static StagingLocation fromString(String s) {
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
  public enum CargoLocation {
    // TODO: add coordinates
    C1(0, 0.0, 0.0), // North of the charging station
    C2(1, 0.0, 0.0), // Centered on northern half of the charging station
    C3(1, 0.0, 0.0), // Centered on southern half of the charging station
    C4(1, 0.0, 0.0); // South of the charging station

    private static final Map<String, CargoLocation> nameMap =
        Map.of("C1", C1, "C2", C2, "C3", C3, "C4", C4);

    private final int id;
    private final double x; // X coordinate
    private final double y; // Y coordinate

    private CargoLocation(int idx, double xLoc, double yLoc) {
      id = idx;
      x = xLoc;
      y = yLoc;
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
}
