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
package frc.robot.subsystems.Heimdall;

import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.AKitLoggers.Pose3dLoggableInput;
import frc.lib.utility.AKitLoggers.Transform3dLoggableInput;
import frc.robot.Constants;
import frc.robot.subsystems.Dashboard.DashboardSubsystem;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Set to true to enable a dashboard tab for the subsystem */
  public static final boolean kDashboardTabIsEnabled = false;

  private PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final Swerve s_swerveSubsystem;
  private final AprilTagFieldLayout ATfieldLayout;

  private final PoseEstimatorDashboardTab m_dashboardTab;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   * and radians.
   */
  private Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, Swerve s_swerveSubsystem) {
    m_dashboardTab = (kDashboardTabIsEnabled) ? new PoseEstimatorDashboardTab(this) : null;
    this.photonCamera = photonCamera;
    this.s_swerveSubsystem = s_swerveSubsystem;
    AprilTagFieldLayout fieldLayout = null;
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Create pose estimator
      photonPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP,
              photonCamera,
              Constants.VisionConstants.CAMERA_TO_ROBOT);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
      fieldLayout = null;
    }
    this.ATfieldLayout = fieldLayout;
    poseEstimator = s_swerveSubsystem.swervePoseEstimator;
    /*poseEstimator =
    new SwerveDrivePoseEstimator(
        s_swerveSubsystem.getSwerveKinematics(),
        s_swerveSubsystem.getYaw(),
        s_swerveSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);*/
  }

  /** Register the subsystem's dashboard tab */
  public void registerDashboardTab(DashboardSubsystem dashboardSubsystem) {
    if (m_dashboardTab != null) {
      dashboardSubsystem.add(m_dashboardTab);
    }
  }

  @Override
  public void periodic() {
    // Update vision processing if executing on the bot
    if (RobotBase.isReal()) {

      // Update pose estimator with the best visible target
      PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
      double resultTimestamp = pipelineResult.getTimestampSeconds();
      if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {

        // Get inputs from PhotonVision
        previousPipelineTimestamp = resultTimestamp;
        PhotonTrackedTarget target = pipelineResult.getBestTarget();

        // Log inputs from PhotonVision
        PhotonTrackedTargetLoggableInputs bestTargetInputLogged =
            new PhotonTrackedTargetLoggableInputs("Heimdall/bestTarget");
        bestTargetInputLogged.update(pipelineResult.getBestTarget(), ATfieldLayout);
        Logger.getInstance().processInputs("Heimdall/bestTarget", bestTargetInputLogged);

        int fiducialId = target.getFiducialId();

        // Log all tracked targets
        for (PhotonTrackedTarget trackedTarget : pipelineResult.getTargets()) {
          int id = trackedTarget.getFiducialId();
          String prefix = String.format("Heimdall/tag%d", id);
          PhotonTrackedTargetLoggableInputs targetInputLogged =
              new PhotonTrackedTargetLoggableInputs(prefix);
          targetInputLogged.update(pipelineResult.getBestTarget(), ATfieldLayout);
          Logger.getInstance().processInputs(prefix, targetInputLogged);
        }

        // Get the tag pose from field layout - consider that the layout will be null if it failed
        // to
        // load
        Optional<Pose3d> tagPose =
            ATfieldLayout == null ? Optional.empty() : ATfieldLayout.getTagPose(fiducialId);
        if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
          var targetPose = tagPose.get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

          var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
          if (target.getPoseAmbiguity() <= .05) {
            visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(2));
          } else {
            visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
          }
          poseEstimator.addVisionMeasurement(
              visionMeasurement.toPose2d(), resultTimestamp, visionMeasurementStdDevs);
        }
      }
    }

    // Update pose estimator with drivetrain sensors
    poseEstimator.update(s_swerveSubsystem.getYaw(), s_swerveSubsystem.getModulePositions());

    // Log the current pose
    Pose2d currentPose = getCurrentPose();
    Logger.getInstance().recordOutput("Heimdall/estimatedPose", currentPose);
    field2d.setRobotPose(currentPose);

    // if (DriverStation.getAlliance() == Alliance.Red) {
    // field2d.setRobotPose(new
    // Pose2d(FieldConstants.fieldLength-getCurrentPose().getX(),FieldConstants.fieldWidth-getCurrentPose().getY(), new Rotation2d(getCurrentPose().getRotation().getRadians()+Math.PI)));
    // } else {
    //  field2d.setRobotPose(getCurrentPose());
    // }
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLog
  private static class SetPoseInput {
    Pose2d pose;
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called when the robot's
   * position on the field is known, like at the beginning of a match.
   *
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    // Log input pose
    SetPoseInputAutoLogged poseInput = new SetPoseInputAutoLogged();
    poseInput.pose = newPose;
    Logger.getInstance().processInputs("Heimdall/setCurrentPoseInput", poseInput);

    poseEstimator.resetPosition(
        s_swerveSubsystem.getYaw(), s_swerveSubsystem.getModulePositions(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public void addTrajectory(PathPlannerTrajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  private static class PhotonTrackedTargetLoggableInputs implements LoggableInputs {
    public String keyPrefix;
    public double yaw;
    public double pitch;
    public double area;
    public double skew;
    public int fiducialId;
    public double poseAmbiguity;
    public Transform3dLoggableInput bestCameraToTarget;
    public Transform3dLoggableInput altCameraToTarget;
    public Pose3dLoggableInput pose3d;

    public PhotonTrackedTargetLoggableInputs(String keyPrefix) {
      keyPrefix = keyPrefix;
      bestCameraToTarget =
          new Transform3dLoggableInput(keyPrefix + "/bestCameraToTarget", new Transform3d());
      altCameraToTarget =
          new Transform3dLoggableInput(keyPrefix + "/alternateCameraToTarget", new Transform3d());
    }

    void update(PhotonTrackedTarget target, AprilTagFieldLayout ATfieldLayout) {
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();
      fiducialId = target.getFiducialId();
      poseAmbiguity = target.getPoseAmbiguity();
      bestCameraToTarget.update(target.getBestCameraToTarget());
      altCameraToTarget.update(target.getAlternateCameraToTarget());

      Optional<Pose3d> tagPose =
          ATfieldLayout == null ? Optional.empty() : ATfieldLayout.getTagPose(fiducialId);
      var targetPose = tagPose.get();
      Transform3d camToTarget = target.getBestCameraToTarget();
      Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

      pose3d.update(camPose.transformBy(CAMERA_TO_ROBOT));
    }

    @Override
    public void toLog(LogTable table) {
      table.put((keyPrefix + "/yaw"), yaw);
      table.put((keyPrefix + "/pitch"), pitch);
      table.put((keyPrefix + "/area"), area);
      table.put((keyPrefix + "/skew"), skew);
      table.put((keyPrefix + "/fiducialId"), fiducialId);
      table.put((keyPrefix + "/poseAmbiguity"), poseAmbiguity);
      bestCameraToTarget.toLog(table);
      ;
      altCameraToTarget.toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
      table.getDouble((keyPrefix + "/yaw"), Double.NaN);
      table.getDouble((keyPrefix + "/pitch"), Double.NaN);
      table.getDouble((keyPrefix + "/area"), Double.NaN);
      table.getDouble((keyPrefix + "/skew"), Double.NaN);
      table.getDouble((keyPrefix + "/fiducialId"), Double.NaN);
      table.getDouble((keyPrefix + "/poseAmbiguity"), Double.NaN);
      bestCameraToTarget.toLog(table);
      ;
      altCameraToTarget.toLog(table);
    }

    public PhotonTrackedTargetLoggableInputs clone() {
      PhotonTrackedTargetLoggableInputs copy =
          new PhotonTrackedTargetLoggableInputs(this.keyPrefix);
      copy.yaw = this.yaw;
      copy.pitch = this.pitch;
      copy.area = this.area;
      copy.skew = this.skew;
      copy.fiducialId = this.fiducialId;
      copy.poseAmbiguity = this.poseAmbiguity;
      copy.bestCameraToTarget = this.bestCameraToTarget.clone();
      copy.altCameraToTarget = this.altCameraToTarget.clone();
      copy.pose3d = this.pose3d.clone();
      return copy;
    }
  }
}
