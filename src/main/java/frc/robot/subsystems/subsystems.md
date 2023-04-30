![FRC 5290 - VIKotics](../../../../../doc/graphics/5920-vikotics-logo_80x80.png "FRC 5290 - VIKotics")
**FRC 5290 - VIKotics**

---

# Robot Subsystems

Robot code that interacts with the various electrical and mechanical devices
that are used to implement robot mechanisms are organized into
[subsystems](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html)
that are accessed by higher-level commands and routines.  A few of these are
documented here.

## [Dashboard Subsystem](./Dashboard/)

The Dashboard Subsystem exists to simplify the process of adding dashboard tabs
to parts of the robot code.  A single instance of the `DashboardSubsystem` can
be created in the robot (e.g. in the `RobotContainer` class).  Once this is
done, a new dashboard tab can be created for a subsystem simply by creating a
class that extends [`IDashboardTab`](Dashboard/IDashboardTab.java) and passing
it to `DashboardSubsystem.add()`.  This will cause the new tab to be added to
the list of Dashboard tabs managed by the subsystem.

When the `initialize()` method is called on the instance of `DashboardSubsystem`,
(usually from the [`robotInit()` method in `Robot.java`](../Robot.java), each
dashboard registered using `add()` will have its `initDashboard()` method called.
After the robot is initialized, the `updateDashboard()` method of each
registered tab gets called during each robot periodic cycle, allowing them to
put or get data from NetworkTables entries if necessary.

## [Heimdall Subsystem](./Heimdall/)

The Heimdall subsystem interacts with PhotonVision and cameras on the robot.
Its [`PoseEstimatorSubsystem`](Heimdall/PoseEstimatorSubsystem.java) implements
estimation of the robot's pose (location on the field) from AprilTags.  This
pose estimation is then passed as a measurement to the primary pose estimator
in the robot's [`SwerveDrivebase`](SwerveDrivebase/Swerve.java) subsystem.

## [Intake Subsystem](./Intake/)

The [`IntakeSubsystem` class](Intake/IntakeSubsystem.java) manages Falcon motors
that drive the compliant wheels of the robot's intake and exposes the state of
the limit switch used to detect the presence of a cube gamepiece.

## [Shooter Pivot Subsystem](./ShooterPivot/)

The [`ShooterPivotSubsystem`](./ShooterPivot/ShooterPivotSubsystem.java) manages
a pair of motors used to pivot the intake mechanism.  It configures closed-loop
control of motor position and a motion profile used to supply smooth movement of
the pivot mechanism.

### Pivot Auto-zeroing

The intake pivot mechanism does not include an absolute encoder.  So, the only
means of measuring the pivot angle is by using the internal sensor of the CTRE
Falcon 2 pivot motors.  Though this sensor offers good resolution for measuring
pivot angle, it accumulates error over time due to small variances and slop
in the pivot mechanism's mechanical linkages.  Consequently, the internal sensor
must be re-zeroed periodically when the pivot is set to its zero degree or
'parked' position.

Re-zeroing of the sensor is performed by an `AutoZeroPivot` internal command
class included with the [`ShooterPivotSubsystem`](./ShooterPivot/ShooterPivotSubsystem.java).
This command is executed whenever the pivot is not in use by another command and
the pivot angle has been changed and then set back to its'parked' position.  The
`AutoZeroPivot` command operates as follows:

1. Read the internal position sensor of the pivot motors.
2. Activate the pivot motors with a small amount of drive percentage to ensure
the pivot moves completely into the 'parked' position.
3. Read the internal position sensor of the pivot motors while the motor is
activated until the change in the sensor reading is below a few sensor ticks.
4. Deactivate the pivot motors and set the motors' internal sensor reading to
zero.

## [Swerve Drivebase Subsystem](./SwerveDrivebase/)

A [`Swerve`](./SwerveDrivebase/Swerve.java) class provides a subsystem that
interfaces with motors and encoders used in four
[Swerve Drive Specialties SDS Mk4i](https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
swerve drive modules.  Each swerve module includes:

* One [CTRE Falcon 500](https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/)
brushless drive motor.
* One [CTRE Falcon 500](https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/)
brushless swerve angle motor.
* One [CTRE CANCoder module](https://store.ctr-electronics.com/cancoder/) used
to provide absolute swerve angle measurement.

Additionally, the [`Swerve`](./SwerveDrivebase/Swerve.java) class includes an
instance of the WPILib `SwerveDrivePoseEstimator` class that is used to estimate
the present pose (position and heading) of the robot on the field based on
odometry measured in the swerve modules and AprilTags measurements processed by
the [Heimdall subsystem](./Heimdall/PoseEstimatorSubsystem.java).

### Swerve Module Calibration

Small inaccuracies in the robot's construction result in swerve module angles
deviating slightly from their ideal position.  During the build phase and
periodically between uses, it is necessary to measure these deviations by
physically moving swerve wheels into parallel alignment and measuring the angle
offset using the CANCoder modules and internal Falcon sensors.  To facilitate
this, the `kDashboardTabIsEnabled` boolean constant in the
[`Swerve`](./SwerveDrivebase/Swerve.java) class can optionally be set to `true`
to enable a dashboard tab that displays sensor readings from the motors and
CANCoders.  Displaying this dashboard tab tends to add processing overhead and
can result in 'laggy' dashboard performance.  So, it is typically disabled
outside of calibration activities.

## [Joystick Subsystem](./JoystickSubsystem.java)

The Joystick subsystem manages up to two Xbox controllers for use in the robot.
Controller buttons are mapped to robot [commands](../commands/commands.md) in
the `configureButtonBindings()` method using typical methods of the WPILib
`Trigger` class (e.g. `OnTrue`, `WhileTrue`, etc.).  Two-button combinations can
be configured using the `setupButtonCombo()` method, allowing an operator to
use buttons for multiple functions or variations of functionality.

The subsystem base class implements deadbanding and sensitivity adjustment of
joysticks and non-binary joystick axes to ensure a reliable experience for
drivers and operators with older, well-used controllers.

## [LED Subsystem](./LEDs.java)

The LED subsystem manages a strip of addressable LEDs attached to the robot and
supplies methods for setting their color or displaying a changing pattern.

## Pneumatics Subsystem

This subsystem is an incomplete implementation carried forward from the original
2023 design that utilized a pneumatic wrist on a robot arm.  It is not used in
the current design.
