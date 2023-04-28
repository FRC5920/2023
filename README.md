![FRC 5290 - VIKotics](./doc/graphics/5920-vikotics-logo_80x80.png "FRC 5290 - VIKotics")
**FRC 5290 - VIKotics**

---

# 2023 Robot Code: Lil' Cheese

This repository contains source code for the VIKotics robot Lil' Cheez fielded
for the 2023 "Charged Up!" FRC competition.

## Repo Organization

![Repo File Tree](./doc/_toplevel-README/repo-file-tree.svg "Repository file tree")

Source Code is divided between the following subdirectories of [src/main/java/frc/](./src/main/java/frc):

- [lib/](./src/main/java/frc/lib): third-party sources and common classes shared across robot
code modules (e.g. Swerve Drive, Joystick, etc.).
- [robot/](./src/main/java/frc/robot): sources that directly implement robot subsystems and runtime routines.

### Robot Code

The primary robot code is centered on a small group of modules located under the
[src/main/java/frc/robot/](./src/main/java/frc/robot) directory:

- [Main.java](./src/main/java/frc/robot/Main.java) - primary code (mostly
boilerplate) serving as an execution entry point
- [Robot.java](./src/main/java/frc/robot/Robot.java) - top-level routines used
to initialize the robot and execute periodic functions for teleop, autonomous,
test, and simulation.
- [RobotContainer.java](./src/main/java/frc/robot/RobotContainer.java) - a class
that contains the instances of all robot subsystems.

Supplemental supporting code is organized under the following subdirectories:

| Subdirectory | Description |
| :----------- | :---------- |
| autos        | Classes used to implement autonomous routines and an associated dashboard tab |
| commands     | Command classes used to carry out robot functions (e.g. Shooter, Z-targeting, etc.) |
| subsystems   | Classes that implement

## Additional Documentation

* [Autonomous Routines](./src/main/java/frc/robot/autos/doc/auto-routines.md)

## Useful Links

- [Discussion about chase tag](https://www.chiefdelphi.com/t/photonvision-beta-2023-apriltags/415626/156?u=amicus1)
- [Chase Tag Command](https://github.com/STMARobotics/swerve-test/blob/b11cc5ab3693b698eed8f6fc38a60813d21e2f45/src/main/java/frc/robot/commands/ChaseTagCommand.java)
- [Mechanical Advantage April Tag Vision](https://github.com/Mechanical-Advantage/RobotCode2023/tree/main/src/main/java/org/littletonrobotics/frc2023/subsystems/apriltagvision)
