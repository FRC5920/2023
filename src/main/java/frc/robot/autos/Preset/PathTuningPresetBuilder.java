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
package frc.robot.autos.Preset;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.utility.TrajectoryLoader;
import frc.robot.RobotContainer;
import frc.robot.commands.Shooter.Shoot.ShootConfig;
import frc.robot.commands.Shooter.ShooterPresets;
import frc.robot.commands.SimulationPrinter;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.HashMap;
import java.util.Set;

/** Add your docs here. */
public class PathTuningPresetBuilder {

  /** Initial pose of the robot in the auto routine */
  private static final Pose2d kInitialPose =
      new Pose2d(2.5, 4.8, new Rotation2d(Units.degreesToRadians(-15.77)));

  /** Configuration used to shoot the initial pre-loaded cube at the beginning of the auto */
  private static final ShootConfig kInitialShotConfig = ShooterPresets.CloseShotLow.config;

  /** Default PID gains applied to translation when following trajectories */
  private static final PIDConstants kDefaultTranslationPIDGains = new PIDConstants(8.0, 0.0, 0.2);
  /** Default PID gains applied to rotation when following trajectories */
  private static final PIDConstants kDefaultRotationPIDGains = new PIDConstants(10.0, 0.0, 0.2);

  /** Max velocity used when following PathPlanner trajectories */
  private static final double kDefaultMaxVelocity = 5.0;

  /** Max acceleration used when following PathPlanner trajectories */
  private static final double kDefaultMaxAcceleration = 5.0;

  private final HashMap<String, TrajectoryLoader> m_trajectoryLoaderMap;

  /** Creates an instance of the builder and loads trajectory files */
  public PathTuningPresetBuilder() {
    String trajectoryNames[] =
        new String[] {"Forward3m", "Forward6m", "Lateral3m", "Lateral6m", "Curve3m"};

    m_trajectoryLoaderMap = new HashMap<>();

    // Load PathPlanner trajectory files
    for (String name : trajectoryNames) {
      String trajectoryFileName = "tuning" + name;
      m_trajectoryLoaderMap.put(
          name,
          new TrajectoryLoader(trajectoryFileName, kDefaultMaxVelocity, kDefaultMaxAcceleration));
    }
  }

  /** Returns an array of Strings identifying preset paths used for tuning */
  public String[] getTrajectoryNames() {
    Set<String> keys = m_trajectoryLoaderMap.keySet();
    return keys.toArray(new String[keys.size()]);
  }

  /**
   * Returns a command that executes the auto routine
   *
   * @param trajectoryName The name of the path tuning trajectory to create a command for
   * @param botContainer Container used to access robot subsystems
   */
  public CommandBase getCommand(String trajectoryName, RobotContainer botContainer) {
    ShooterPivotSubsystem shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intakeSubsystem = botContainer.intakeSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    TrajectoryLoader trajectoryLoader = m_trajectoryLoaderMap.get(trajectoryName);
    String logName = String.format("<PathTuning: %s>", trajectoryName);
    HashMap<String, Command> emptyEventMap = new HashMap<>();

    CommandBase autoCommands =
        Commands.sequence(
            // First, a command to reset the robot pose to the initial position
            new SimulationPrinter(logName + " Set initial pose"),
            new InstantCommand(
                () -> {
                  swerveSubsystem.zeroGyro();
                  swerveSubsystem.resetOdometry(kInitialPose);
                  botContainer.poseEstimatorSubsystem.setCurrentPose(kInitialPose);
                  // swerveSubsystem.setWheelPreset(WheelPreset.Forward);
                }),
            // Drive the preset trajectory
            new SimulationPrinter(logName + " move to acquire C1"),
            trajectoryLoader.generateTrajectoryCommand(
                emptyEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new SimulationPrinter(logName + " trajectory completed"));
    return autoCommands;
  }

  public Pose2d getInitialPose(String trajectoryName) {
    TrajectoryLoader trajectoryLoader = m_trajectoryLoaderMap.get(trajectoryName);
    return trajectoryLoader.getInitialPose();
  }

  /** Returns a list containing trajectories used to illustrate motion in the auto routine */
  public PathPlannerTrajectory getTrajectory(String trajectoryName) {
    TrajectoryLoader trajectoryLoader = m_trajectoryLoaderMap.get(trajectoryName);
    return trajectoryLoader.getTrajectory();
  }
}
