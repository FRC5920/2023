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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.thirdparty.FRC6328.AllianceFlipUtil;
import frc.lib.utility.BotBoundary.Polygon;
import frc.lib.utility.BotBoundary.PoseLimiter;
import frc.lib.utility.BotBoundary.PoseLimiter.BoundaryPolicy;
import frc.lib.utility.BotLog;
import frc.lib.utility.TrajectoryLoader;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.CargoLocation;
import frc.robot.autos.AutoConstants.FieldCoordinates;
import frc.robot.commands.Balance;
import frc.robot.commands.Shooter.SetShooterAngle;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.Shoot.ShootConfig;
import frc.robot.commands.zTarget.AutoIntakeWithZTargeting;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivot.PivotPresets;
import frc.robot.subsystems.ShooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class NorthLinkOverCSPresetBuilder {

  /** Configuration used to shoot the initial pre-loaded cube at the beginning of the auto */
  private static final ShootConfig kInitialShotConfig = new ShootConfig(20, 22.5);

  private static final ShootConfig kCloseShotConfig = new ShootConfig(20, 15);

  private static final ShootConfig kBalanceShootConfig = new ShootConfig(20, 50);

  /** Holonomic rotation to use when balancing */
  private static final Rotation2d kBalanceRotation = Rotation2d.fromDegrees(170.0);

  /** Default PID gains applied to translation when following trajectories */
  private static final PIDConstants kDefaultTranslationPIDGains = new PIDConstants(8.0, 0.0, 0.2);
  /** Default PID gains applied to rotation when following trajectories */
  private static final PIDConstants kDefaultRotationPIDGains = new PIDConstants(10.0, 0.0, 0.2);

  /** Max velocity used when following PathPlanner trajectories */
  private static final double kDefaultMaxVelocity = 5.0;

  /** Max acceleration used when following PathPlanner trajectories */
  private static final double kDefaultMaxAcceleration = 5.0;

  /** PathPlanner trajectory file and configuration used to load C1 */
  private final TrajectoryLoader m_NorthOverCSAcquireC1Loader;
  /** PathPlanner trajectory file and constraints used to shoot C1 */
  private final TrajectoryLoader m_NorthOverCSShootC1Loader;
  /** PathPlanner trajectory file and constraints used to load C2 */
  private final TrajectoryLoader m_NorthOverCSAcquireC2Loader;
  /** PathPlanner trajectory file and constraints used to mount the charging station */
  private final TrajectoryLoader m_NorthOverCSMountCSLoader;

  /** Creates an instance of the builder and loads trajectory files */
  public NorthLinkOverCSPresetBuilder() {
    // Load PathPlanner trajectory files
    m_NorthOverCSAcquireC1Loader =
        new TrajectoryLoader("NorthLOCS_0acquireC1", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_NorthOverCSShootC1Loader =
        new TrajectoryLoader("NorthLOCS_1shootC1", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_NorthOverCSAcquireC2Loader =
        new TrajectoryLoader("NorthLOCS_2acquireC2", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_NorthOverCSMountCSLoader =
        new TrajectoryLoader("NorthLOCS_3mountCS", kDefaultMaxVelocity, kDefaultMaxAcceleration);
  }

  /**
   * Returns a command that executes the auto routine
   *
   * @param autoType The type of preset auto to build
   * @param botContainer Container used to access robot subsystems
   */
  public CommandBase getCommand(RobotContainer botContainer) {
    ShooterPivotSubsystem shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intakeSubsystem = botContainer.intakeSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    HashMap<String, Command> intakeEventMap = new HashMap<>();
    intakeEventMap.put(
        "deployPivot", new SetShooterAngle(shooterPivotSubsystem, PivotPresets.Acquire));

    PoseLimiter c1AcquireBoundary = makeAcquireBoundary(CargoLocation.C1, swerveSubsystem::getPose);
    PoseLimiter c2AcquireBoundary = makeAcquireBoundary(CargoLocation.C2, swerveSubsystem::getPose);
    String autoName = "<Link+Balance over CS>";
    Pose2d initialPose = getInitialPose();

    CommandBase autoCommands =
        Commands.sequence(
            // First, a command to reset the robot pose to the initial position
            new BotLog.PrintCommand("Set initial pose"),
            new InstantCommand(
                () -> {
                  swerveSubsystem.resetGyro(AllianceFlipUtil.apply(BotOrientation.kFacingGrid));
                  swerveSubsystem.resetOdometry(initialPose);
                  botContainer.poseEstimatorSubsystem.setCurrentPose(initialPose);
                }),
            // Shoot pre-loaded cube
            new BotLog.PrintCommand(autoName + " shoot pre-loaded cargo"),
            new Shoot(kInitialShotConfig, shooterPivotSubsystem, intakeSubsystem),
            // Move to and acquire C1
            new BotLog.PrintCommand(autoName + " move to acquire C1"),
            m_NorthOverCSAcquireC1Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            // Use vision and Z-targeting to intake C1
            new BotLog.PrintCommand(autoName + " acquire C1"),
            new AutoIntakeWithZTargeting(
                GameTarget.Cube,
                botContainer.ArmCamera,
                swerveSubsystem,
                shooterPivotSubsystem,
                intakeSubsystem,
                c1AcquireBoundary),
            // Move and shoot C1
            new BotLog.PrintCommand(autoName + " move to shoot C1"),
            m_NorthOverCSShootC1Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new BotLog.PrintCommand(autoName + " shoot C1"),
            new Shoot(kCloseShotConfig, shooterPivotSubsystem, intakeSubsystem),
            // Move to and acquire C2
            new BotLog.PrintCommand(autoName + " move to acquire C2"),
            m_NorthOverCSAcquireC2Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            // Use vision and Z-targeting to intake C2
            new BotLog.PrintCommand(autoName + " acquire C2"),
            new AutoIntakeWithZTargeting(
                GameTarget.Cube,
                botContainer.ArmCamera,
                swerveSubsystem,
                shooterPivotSubsystem,
                intakeSubsystem,
                c2AcquireBoundary),
            // Move and shoot C2
            new BotLog.PrintCommand(autoName + " mount CS to shoot C2"),
            m_NorthOverCSMountCSLoader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new BotLog.PrintCommand(autoName + " Balance on Charging Station"),
            new Balance(swerveSubsystem, AllianceFlipUtil.apply(kBalanceRotation)),
            new BotLog.PrintCommand(autoName + " shoot C3 and balance"),
            Commands.parallel(
                new Shoot(kBalanceShootConfig, shooterPivotSubsystem, intakeSubsystem),
                new Balance(swerveSubsystem)));
    return autoCommands;
  }

  public Pose2d getInitialPose() {
    return m_NorthOverCSAcquireC1Loader.getInitialPose();
  }

  /** Returns a list containing trajectories used to illustrate motion in the auto routine */
  public List<PathPlannerTrajectory> getTrajectories() {
    // Return trajectories for display
    List<PathPlannerTrajectory> trajectories = new ArrayList<>();
    trajectories.clear();
    trajectories.add(m_NorthOverCSAcquireC1Loader.getTrajectory());
    trajectories.add(m_NorthOverCSShootC1Loader.getTrajectory());
    trajectories.add(m_NorthOverCSAcquireC2Loader.getTrajectory());
    trajectories.add(m_NorthOverCSMountCSLoader.getTrajectory());

    return trajectories;
  }

  public static PoseLimiter makeAcquireBoundary(
      CargoLocation gamepiece, Supplier<Pose2d> poseSupplier) {
    // Get the position of the gamepiece (translated to current alliance)
    Translation2d loc = gamepiece.getLocation();
    Polygon boundary = null;

    switch (DriverStation.getAlliance()) {
      case Blue:
        boundary =
            Polygon.simpleRectangle(
                new Translation2d(FieldCoordinates.xMin, FieldCoordinates.yMax),
                new Translation2d(FieldCoordinates.xCenterline - 0.5, loc.getY() - 0.75));
        break;

      case Red:
        boundary =
            Polygon.simpleRectangle(
                new Translation2d(FieldCoordinates.xCenterline + 0.5, FieldCoordinates.yMax),
                new Translation2d(FieldCoordinates.xMax, loc.getY() - 0.75));
        break;
      case Invalid:
        throw new RuntimeException("Invalid alliance selected");
    }

    return new PoseLimiter(poseSupplier, boundary, BoundaryPolicy.KeepInside);
  }
}
