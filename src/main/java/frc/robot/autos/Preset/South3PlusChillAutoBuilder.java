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
import frc.lib.utility.BotLogger.BotLog;
import frc.lib.utility.TrajectoryLoader;
import frc.robot.Constants.GameTarget;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.BotOrientation;
import frc.robot.autos.AutoConstants.CargoLocation;
import frc.robot.autos.AutoConstants.FieldCoordinates;
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

/**
 * South3PlusChillAutoBuilder is an object used to build a command that will carry out the `South
 * 3+chill` autonomous routine
 */
public class South3PlusChillAutoBuilder implements PresetBuilder {

  /** Configuration used to shoot the initial pre-loaded cube at the beginning of the auto */
  private static final ShootConfig kCableProtectorShootConfig = new ShootConfig(20, 40);

  private static final ShootConfig kShootC3Config = new ShootConfig(20, 65);

  /** Default PID gains applied to translation when following trajectories */
  private static final PIDConstants kDefaultTranslationPIDGains = new PIDConstants(8.0, 0.0, 0.2);
  /** Default PID gains applied to rotation when following trajectories */
  private static final PIDConstants kDefaultRotationPIDGains = new PIDConstants(10.0, 0.0, 0.2);

  /** Max velocity used when following PathPlanner trajectories */
  private static final double kDefaultMaxVelocity = 5.0;

  /** Max acceleration used when following PathPlanner trajectories */
  private static final double kDefaultMaxAcceleration = 5.0;

  /** PathPlanner trajectory file and configuration used to load C4 */
  private final TrajectoryLoader m_acquireC4Loader;
  /** PathPlanner trajectory file and constraints used to shoot C4 */
  private final TrajectoryLoader m_shootC4Loader;
  /** PathPlanner trajectory file and constraints used to load C3 */
  private final TrajectoryLoader m_acquireC3Loader;
  /** PathPlanner trajectory file and constraints used to shoot C3 */
  private final TrajectoryLoader m_shootC3Loader;

  /** Creates an instance of the builder and loads trajectory files */
  public South3PlusChillAutoBuilder() {
    // Load PathPlanner trajectory files
    m_acquireC4Loader =
        new TrajectoryLoader("SouthLNB_0acquireC4", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_shootC4Loader =
        new TrajectoryLoader("SouthLNB_1shootC4", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_acquireC3Loader =
        new TrajectoryLoader("SouthLNB_2acquireC3", kDefaultMaxVelocity, kDefaultMaxAcceleration);
    m_shootC3Loader =
        new TrajectoryLoader("SouthLNChill_3shootC3", kDefaultMaxVelocity, kDefaultMaxAcceleration);
  }

  /**
   * Returns a command that executes the auto routine
   *
   * @param autoType The type of preset auto to build
   * @param botContainer Container used to access robot subsystems
   */
  @Override
  public CommandBase getCommand(RobotContainer botContainer) {
    ShooterPivotSubsystem shooterPivotSubsystem = botContainer.shooterPivotSubsystem;
    IntakeSubsystem intakeSubsystem = botContainer.intakeSubsystem;
    Swerve swerveSubsystem = botContainer.swerveSubsystem;

    HashMap<String, Command> intakeEventMap = new HashMap<>();
    intakeEventMap.put(
        "deployPivot", new SetShooterAngle(shooterPivotSubsystem, PivotPresets.Acquire));

    PoseLimiter c1AcquireBoundary = makeAcquireBoundary(CargoLocation.C1, swerveSubsystem::getPose);
    PoseLimiter c2AcquireBoundary = makeAcquireBoundary(CargoLocation.C2, swerveSubsystem::getPose);
    String autoName = "<South Link+Chill>";
    Pose2d initialPose = getInitialPose();

    CommandBase autoCommands =
        Commands.sequence(
            // First, a command to reset the robot pose to the initial position
            new BotLog.InfoPrintCommand(autoName + " Set initial pose"),
            new InstantCommand(
                () -> {
                  swerveSubsystem.resetGyro(AllianceFlipUtil.apply(BotOrientation.kFacingGrid));
                  swerveSubsystem.resetOdometry(initialPose);
                  botContainer.poseEstimatorSubsystem.setCurrentPose(initialPose);
                }),
            // Shoot pre-loaded cube
            new BotLog.InfoPrintCommand(autoName + " shoot pre-loaded cargo"),
            new Shoot(kCableProtectorShootConfig, shooterPivotSubsystem, intakeSubsystem),
            // Move to and acquire C1
            new BotLog.InfoPrintCommand(autoName + " move to acquire C4"),
            m_acquireC4Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            // Use vision and Z-targeting to intake C1
            new BotLog.InfoPrintCommand(autoName + " acquire C4"),
            new AutoIntakeWithZTargeting(
                GameTarget.Cube,
                botContainer.ArmCamera,
                swerveSubsystem,
                shooterPivotSubsystem,
                intakeSubsystem,
                c1AcquireBoundary),
            // Move and shoot C1
            new BotLog.InfoPrintCommand(autoName + " move to shoot C4"),
            m_shootC4Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new BotLog.InfoPrintCommand(autoName + " shoot C4"),
            new Shoot(kCableProtectorShootConfig, shooterPivotSubsystem, intakeSubsystem),
            // Move to and acquire C2
            new BotLog.InfoPrintCommand(autoName + " move to acquire C3"),
            m_acquireC3Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            // Use vision and Z-targeting to intake C2
            new BotLog.InfoPrintCommand(autoName + " acquire C3"),
            new AutoIntakeWithZTargeting(
                GameTarget.Cube,
                botContainer.ArmCamera,
                swerveSubsystem,
                shooterPivotSubsystem,
                intakeSubsystem,
                c2AcquireBoundary),
            // Move and shoot C2
            new BotLog.InfoPrintCommand(autoName + " move to shoot C3"),
            m_shootC3Loader.generateTrajectoryCommand(
                autoName,
                intakeEventMap,
                swerveSubsystem,
                kDefaultTranslationPIDGains,
                kDefaultRotationPIDGains,
                new PathConstraints(kDefaultMaxVelocity, kDefaultMaxAcceleration)),
            new BotLog.InfoPrintCommand(autoName + " Shoot C3"),
            new Shoot(kShootC3Config, shooterPivotSubsystem, intakeSubsystem));
    return autoCommands;
  }

  @Override
  public Pose2d getInitialPose() {
    return m_acquireC4Loader.getInitialPose();
  }

  /** Returns a list containing trajectories used to illustrate motion in the auto routine */
  @Override
  public List<PathPlannerTrajectory> getTrajectories() {

    // Return trajectories for display
    List<PathPlannerTrajectory> trajectories = new ArrayList<>();
    trajectories.clear();
    trajectories.add(m_acquireC4Loader.getTrajectory());
    trajectories.add(m_shootC4Loader.getTrajectory());
    trajectories.add(m_acquireC3Loader.getTrajectory());
    trajectories.add(m_shootC3Loader.getTrajectory());

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
