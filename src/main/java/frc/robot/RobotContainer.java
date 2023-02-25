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
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.SwerveDrive.Falcon500SwerveIO;
import frc.lib.SwerveDrive.GyroIO;
import frc.lib.SwerveDrive.Pigeon2GyroIO;
import frc.lib.SwerveDrive.SimGyroIO;
import frc.lib.SwerveDrive.SimSwerveModuleIO;
import frc.lib.SwerveDrive.SwerveModuleIO;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.RotateIntake;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Pneumatics;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;
import frc.robot.subsystems.Heimdall.*;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Pneumatics.Pneumatics;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should` be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // --------------------- Robot Subsystems ----------------------------
  public final DashboardSubsystem dashboardSubsystem = new DashboardSubsystem();
  public final JoystickSubsystem joystickSubsystem = new JoystickSubsystem();
  public final BotStateSubsystem botStateSubsystem = new BotStateSubsystem();
  public final Swerve swerveSubsystem;
  public final Intake m_Intake = new Intake();
  public final Pneumatics s_Pneumatics = new Pneumatics();

  @SuppressWarnings({"unused"})
  public final PoseEstimatorSubsystem poseEstimatorSubsystem;
  /* Drive Controls */
  public final int translationAxis = XboxController.Axis.kLeftY.value;
  public final int strafeAxis = XboxController.Axis.kLeftX.value;
  public final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Cameras */
  private final PhotonCamera TagCamera = new PhotonCamera(Constants.VisionConstants.TagCameraName);

  @SuppressWarnings({"unused"})
  private final PhotonCamera BackCamera =
      new PhotonCamera(Constants.VisionConstants.BackupCameraName);
  public final PhotonCamera ArmCamera = new PhotonCamera(Constants.VisionConstants.ArmCameraName);

  // --------------------- Robot Subsystems ----------------------------
  public final JoystickSubsystem joystickSubsystem = new JoystickSubsystem();
  public final Swerve swerveSubsystem = new Swerve();
  public final BotStateSubsystem s_BotState = new BotStateSubsystem();
  private final PoseEstimatorSubsystem s_poseEstimator =
      new PoseEstimatorSubsystem(TagCamera, swerveSubsystem);

  public final Pneumatics s_Pneumatics = new Pneumatics();
  public final Arm s_Arm = new Arm(s_Pneumatics);

  /* Dashboard Subsystems */
  public final DriveTab s_DriveTab = new DriveTab();

  // The robot's subsystems and commands are defined here...
  // private final Balance m_Balance = new Balance(swerveSubsystem, s_poseEstimator);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public boolean fieldRelative;

  public boolean openLoop;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //    new CommandXboxController(DriverConstants.kControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    SwerveModuleIO swerveModuleIO[];
    GyroIO gyroIO;

    // Instantiate active subsystems
    switch (Constants.getMode()) {
      case REAL:
        gyroIO = new Pigeon2GyroIO(Constants.SwerveDrivebaseConstants.pigeonID, "SwerveCAN");
        swerveModuleIO =
            new SwerveModuleIO[] {
              new Falcon500SwerveIO(
                  Constants.SwerveDrivebaseConstants.Mod0.driveMotorID,
                  Constants.SwerveDrivebaseConstants.Mod0.angleMotorID,
                  Constants.SwerveDrivebaseConstants.Mod0.canCoderID,
                  Constants.SwerveDrivebaseConstants.Mod0.angleOffset,
                  Robot.ctreConfigs),
              new Falcon500SwerveIO(
                  Constants.SwerveDrivebaseConstants.Mod1.driveMotorID,
                  Constants.SwerveDrivebaseConstants.Mod1.angleMotorID,
                  Constants.SwerveDrivebaseConstants.Mod1.canCoderID,
                  Constants.SwerveDrivebaseConstants.Mod1.angleOffset,
                  Robot.ctreConfigs),
              new Falcon500SwerveIO(
                  Constants.SwerveDrivebaseConstants.Mod2.driveMotorID,
                  Constants.SwerveDrivebaseConstants.Mod2.angleMotorID,
                  Constants.SwerveDrivebaseConstants.Mod2.canCoderID,
                  Constants.SwerveDrivebaseConstants.Mod2.angleOffset,
                  Robot.ctreConfigs),
              new Falcon500SwerveIO(
                  Constants.SwerveDrivebaseConstants.Mod3.driveMotorID,
                  Constants.SwerveDrivebaseConstants.Mod3.angleMotorID,
                  Constants.SwerveDrivebaseConstants.Mod3.canCoderID,
                  Constants.SwerveDrivebaseConstants.Mod3.angleOffset,
                  Robot.ctreConfigs)
            };
        break;

      case SIM:
      case REPLAY:
      default:
        gyroIO = new SimGyroIO();
        swerveModuleIO =
            new SwerveModuleIO[] {
              new SimSwerveModuleIO(),
              new SimSwerveModuleIO(),
              new SimSwerveModuleIO(),
              new SimSwerveModuleIO()
            };
        break;
    }

    swerveSubsystem =
        new Swerve(
            Constants.SwerveDrivebaseConstants.trackWidth,
            Constants.SwerveDrivebaseConstants.wheelBase,
            gyroIO,
            swerveModuleIO[Swerve.ModuleId.kFrontLeft.value],
            swerveModuleIO[Swerve.ModuleId.kFrontRight.value],
            swerveModuleIO[Swerve.ModuleId.kRearLeft.value],
            swerveModuleIO[Swerve.ModuleId.kRearRight.value]);

    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem,
            joystickSubsystem.driverController,
            translationAxis,
            strafeAxis,
            rotationAxis,
            fieldRelative,
            openLoop));

    swerveSubsystem.registerDashboardTab(dashboardSubsystem);

    poseEstimatorSubsystem = new PoseEstimatorSubsystem(TagCamera, swerveSubsystem);
    poseEstimatorSubsystem.registerDashboardTab(dashboardSubsystem);

    // Initialize all dashboard tabs
    dashboardSubsystem.initialize(this);

    // Configure joystick button bindings
    joystickSubsystem.configureButtonBindings(this);

    joystickSubsystem.configureButtonBindings(this);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
