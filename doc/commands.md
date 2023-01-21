# 2023 Commands

FRC 5290: VIKotics

## Decomposition of Autonomous Routine Into Commands

A basic autonomous routine is presented in which the robot executes the following sequence of actions:

1. Place a pre-loaded game piece on the nearest goal.
2. Drive to and collect a second game piece.
3. Deliver the second game piece to another goal.
4. Drive onto the Charging Station (CS)
5. Balance the Charging Station

This series of actions can be decomposed into a series of required behaviors and corresponding commands
used to carry out the desired actions.  These commands are shown in Figure 1.

__Fig 1: Command sequence for example autonomous routine__
![Command sequence for example auto routine](./_commands/basic-auto-routine-commands.svg "Command sequence for example autonomous routine")

## Commands

### Place Cargo

This command executes the actions needed to make the robot place a game piece it is carrying on a given
goal.  It is represented in both steps 1 and 7 since placement actions are needed for placing both game
pieces.  Notably, while the command implementation might be reused, it may need to be parameterized to
accommodate placement of different game piece types (e.g. a cube in Step 1 and a cone in Step 7).

### Fetch

In this command, the robot follows a predefined path that will take it within close proximity of a
target game piece.  When the robot reaches the end of this path, it should be in a position that
places the target game piece within the detection range of the robot's vision subsystem.  At that
point, the robot transitions to the __Align + Intake__ command, ideally without the robot coming
to a complete stop at the end of the __Fetch__ path.

### Align + Intake

When this command is operational, vision processing will be used to identify a target game piece, align
the robot with it, and pick it up using the robot's intake system.  (Sounds easy, right?).  The command
is considered complete when a game piece has been acquired successfully.  Consequently, this strategy is
predicated on the robot's ability to correctly identify whether it has acquired a game piece.  If game
piece acquisition fails, an alternative strategy is needed.  For example, the robot might re-attempt the
__Align + Intake__ command repeatedly until it is successful.

### Go To Delivery

Since the position of the robot in the arena following intake and game piece acquisition cannot be known
precisely _a priori_, this command may be needed as a distinct, intermediate stage that drives the robot
to the starting position of the predefined _delivery path_ used to transport the robot's game piece to a
target goal.  In this command a path must be dynamically generated from the robot's present position to
the initial coordinates of the trajectory followed in the __Deliver__ command.

### Deliver

This command leads the robot along a predefined path that takes it within proximity of a desired target
goal.  Similar to the __Fetch__ command, when the robot reaches the end of the _delivery path_, it must
be in a position and heading that places the desired target within the range of the vision subsystems.

### Align Goal

In this command, the robot's vision subsystems detect the target goal and align the robot to center on
it and move into a consistent position for game piece placement.  Notably, different vision targets
will likely be required for cones and cubes.  Cones will require alignment of the robot with posts,
while cubes will require alignment of the robot with an AprilTag.

### Climb CS

This command moves the robot from its present position onto the Charging Station.  This may require a
controlled approach in order to accommodate the varying slope of the charging station.  It may also
require the robot to identify when the proximal side of the charging station is presenting an
un-climbable slope (e.g. due to another robot climbing it on the opposite side).

### Balance CS

This command executes when the robot has successfully boarded the charging station.  It must vary the
robot's X position on the charging station in conjunction with gyro readings in order to reduce the
gyro slope to zero, indicating a balanced charging station.  Additionally, this command must also
constrain the motion of the robot such that it does not move off of the charging station.  If gyro
readings are subject to inaccuracy or drift over the course of the match, it may be necessary for the
robot to detect when the charging station is lit, indicating the balance point.

## References

- [FRC 2023 Game animation](https://www.youtube.com/watch?v=0zpflsYc4PA&feature=youtu.be)
- [Charged Up Arena](https://firstfrc.blob.core.windows.net/frc2023/Manual/Sections/2023FRCGameManual-05.pdf)
- [Charged Up Match Play](https://firstfrc.blob.core.windows.net/frc2023/Manual/Sections/2023FRCGameManual-06.pdf)
