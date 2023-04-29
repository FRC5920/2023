![FRC 5290 - VIKotics](../../../../../../doc/graphics/5920-vikotics-logo_80x80.png "FRC 5290 - VIKotics")
**FRC 5290 - VIKotics**

---

# Commands

The 2023 robot software design makes extensive use of
[command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)
to decompose complex robot behaviors into smaller, simpler routines that supply
basic functionality.  Several of these commands are described below.

## [`TeleopSwerve`](./TeleopSwerve.java)

The `TeleopSwerve` command allows an operator to drive the robot manually using
the joysticks on a connected Xbox controller.  This command is set up as the
default command for the `SwerveDriveBase` subsystem, and is active anytime the
robot is placed in tele-operated (TeleOp) mode from the Driver Station.  When
the command is active, its `execute()` method gets called each time the robot
processing loop executes.  This method reads the positions of the left and right
joysticks on the Xbox controller and uses them to produce corresponding speeds
indicating motion for translation (forward/backward and left/right motion) and
rotation of the robot about its center.  The speeds are then passed to the
`Swerve` subsystem's `drive()` method.  Notably, the speeds are requested as
open-loop and are given as robot-relative rather than field-relative.

## [`Balance`](./Balance.java)

The `Balance` command is intended to be activated once the robot has moved onto
the Charge Station, in either the Autonomous or TeleOp period of a match.  When
active, the `execute()` method of this command gets called each iteration of the
robot processing loop.  The command reads the robot's pitch, roll, and yaw and
feeds them to three corresponding PID controllers.  The PID controllers are
configured to produce control efforts needed to reduce pitch and roll to zero
and yaw to a predefined value.  The control outputs of the PID controllers are
translated into translation and rotation speeds that are passed to the `Swerve`
subsystem's `drive()` method.

`Balance` command instances can be created to balance perpetually (typically at
the end of a match), in which case the command never finishes.  Alternatively,
a `Balance` command can be created such that it ends once a balance has been
achieved, e.g. allowing subsequent operations such as shooting a cube.

## Z-targeting Commands

Two commands are provided that implement "Z-targeting": a behavior in which the
robot's position on the field is controlled by a joystick or autonomous routine
while onboard vision processing and object detection is used to control the
robot's holonomic rotation such that a detected gamepiece is kept centered in
front of the robot.  Detection of gamepieces and control of holonomic rotation
are provided by a common [`ZTargeter`](ls ../../lib/utility/ZTargeter.java)
class in the `/lib/utils` directory.  A `ZTargeter` object uses a provided
PhotonVision camera to detect gamepieces.  When a gamepiece is detected, a PID
controller is used to produce a control law that reduces the angle between the
gamepiece and the center of the camera's field-of-view (FOV) to zero.  The
center of the camera FOV is assumed to be coincident with the robot's X-axis.
The Z-targeter control law is used to control the holonomic rotation of the
drivebase, allowing the robot to be moved about the field while rotating
automatically to keep facing the detected gamepiece.

The first Z-targeting command, [`DriveWithZTargeting`](./zTarget/DriveWithZTargeting.java),
provides a variation of the `TeleOpSwerve` command in which holonomic rotation
of the robot is nominally controlled by an operator via joystick.  When a
gamepiece is detected by the PhotonVision camera, joystick control of rotation
is replaced by rotation control from a `ZTargeter`.  This command is intended
for use in TeleOp mode, where it can help to reduce the effort required from a
driver to intake a gamepiece.

Another command, `AutoIntakeWithZTargeting`(./zTarget/AutoIntakeWithZTargeting.java),
offers similar functionality for use in Autonomous routines.  In this command,
Z-targeting is first used to adjust holonomic rotation of the robot to be
centered on a detected gamepiece.  Once centering is finished, the robot's
intake system is engaged and it is moved forward until the gamepiece has been
acquired or the robot's pose on the field exceeds a given boundary.

## Shooter Commands

Several commands are provided for controlling the robot's Shooter/Pivot.  These
include:

### [`AcquireAndPark`](./Shooter/AcquireAndPark.java)

This command implements the following steps:

1. Position the shooter/pivot mechanism at an angle used to intake a cube.
2. Run the intake wheels until acquisition of a cube is detected.
3. Move the shooter/pivot mechanism back to its 'park' position.

### [`Shoot`](./Shooter/Shoot.java)

This command implements shooting a cube that is assumed to be present in the
robot's shooter/pivot mechanism.  Primarily, instances of this command use a
`ShootConfig` object as a parameter that gives an angle and shooter speed.
Experimentally-determined angles and speeds for various goals are provided in
the [`ShooterPresets` enum](./Shooter/ShooterPresets.java), and variations on
the `Shoot` constructor allow arbitrary values to be used for a given command
instance.

### [`IntakeGamepiece`](./Shooter/IntakeGamepiece.java)

This command is used to support the `AcquireAndPark` command.  it implements
running the intake wheels until a gamepiece is detected in the intake mechanism.
Detection of a gamepiece is done via a limit switch placed at the end of the
intake mechanism.  However, code exists from earlier iterations of the design in
which a gamepiece was sensed by using averaging filters to detect when average
motor current exceeds a threshold or average motor speed dips below a given
value.

### [`SetShooterAngle`](./Shooter/SetShooterAngle.java)

The `SetShooterAngle` command is used to move the shooter/pivot mechanism to a
given angle.  The command completes when the mechanism has reached the commanded
angle.

## [SnapToGrid](./SnapToGrid.java)

The `SnapToGrid` command provides a version of `TeleOpSwerve` that automatically
detects when the robot is within a short 'capture distance' of the Y-coordinate
of a cube Grid and adjusts commanded speeds to move to and along that
Y-coordinate.  The effect is similar to the "snap-to-grid" option found in many
computerized drawing programs.  The command can optionally draw grid lines on
a Dashboard `Field2d` object using trajectories when it is active.
