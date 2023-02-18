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
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.subsystems.SwerveDrivebase.Swerve;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final Swerve s_swerveSubsystem;
  private final AprilTagFieldLayout ATfieldLayout;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state
   * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
   * then meters.
   */
  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   * and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, Swerve s_swerveSubsystem) {
    this.photonCamera = photonCamera;
    this.s_swerveSubsystem = s_swerveSubsystem;
    AprilTagFieldLayout fieldLayout = null;
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Set origin (Don't know if we need this anymore)
      var alliance = DriverStation.getAlliance();
      // var alliance = Alliance.Blue;
      fieldLayout.setOrigin(
          alliance == Alliance.Blue
              ? OriginPosition.kBlueAllianceWallRightSide
              : OriginPosition.kRedAllianceWallRightSide);
      // End of setting origin
      // Create pose estimator
      photonPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.LOWEST_AMBIGUITY,
              photonCamera,
              Constants.VisionConstants.CAMERA_TO_ROBOT);
      // photonPoseEstimator =
      //          new PhotonPoseEstimator(
      //                 fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera,
      // Constants.VisionConstants.CAMERA_TO_ROBOT);
      //        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
      fieldLayout = null;
    }
    this.ATfieldLayout = fieldLayout;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveDrivebaseConstants.swerveKinematics,
            s_swerveSubsystem.getGyroscopeRotation(),
            s_swerveSubsystem.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to
      // load
      Optional<Pose3d> tagPose =
          ATfieldLayout == null ? Optional.empty() : ATfieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        s_swerveSubsystem.getGyroscopeRotation(), s_swerveSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
    // if (DriverStation.getAlliance() == Alliance.Red) {
    // field2d.setRobotPose(new
    // Pose2d(FieldConstants.fieldLength-getCurrentPose().getX(),FieldConstants.fieldWidth-getCurrentPose().getY(), new Rotation2d(getCurrentPose().getRotation().getRadians()+Math.PI)));
    // } else {
    //  field2d.setRobotPose(getCurrentPose());
    // }
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called when the robot's
   * position on the field is known, like at the beginning of a match.
   *
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        s_swerveSubsystem.getGyroscopeRotation(), s_swerveSubsystem.getModulePositions(), newPose);
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
}
