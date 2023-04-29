![FRC 5290 - VIKotics](../../../../../doc/graphics/5920-vikotics-logo_80x80.png "FRC 5290 - VIKotics")
**FRC 5290 - VIKotics**

---

### Robot Code

The primary robot code is centered on a small group of modules located under the
[src/main/java/frc/robot/](./) directory:

- [Main.java](./Main.java) - primary code (mostly
boilerplate) serving as an execution entry point
- [Robot.java](./Robot.java) - top-level routines used
to initialize the robot and execute periodic functions for teleop, autonomous,
test, and simulation.
- [RobotContainer.java](./RobotContainer.java) - a class
that contains the instances of all robot subsystems.

These modules are supplemented and supported by [code modules under the /lib](../lib/lib-code.md)
directory and modules in the following subdirectories:

| Subdirectory | Description |
| :----------- | :---------- |
| autos        | Classes used to implement autonomous routines and an associated dashboard tab |
| commands     | Command classes used to carry out robot functions (e.g. Shooter, Z-targeting, etc.) |
| subsystems   | Classes that implement robot subsystems (swerve drivebase, shooter, etc.)
