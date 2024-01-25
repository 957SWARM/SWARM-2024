package com.team957.comp2024;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class ATPoseEstimation {

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    SwerveModulePosition[] modulePositions;
    Rotation2d gyro;

    boolean robotEnabled;

    AprilTagFieldLayout ATLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonCamera LL1 = new PhotonCamera("limelight"); // CHECK CAMERA NAME
    // PhotonCamera LL2 = new PhotonCamera("limelight2");

    SwerveDrivePoseEstimator poseEstimator;

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            ATLayout, VisionConstants.POSE_STRATEGY, LL1, VisionConstants.LL1_TO_CENTER);

    public ATPoseEstimation(
            SwerveDriveKinematics kinematics,
            SwerveModulePosition[] modulePositions,
            Rotation2d gyro,
            boolean robotEnabled) {

        this.kinematics = kinematics;
        this.modulePositions = modulePositions;
        this.gyro = gyro;
        this.robotEnabled = robotEnabled;

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyro,
                modulePositions,
                new Pose2d(),
                VisionConstants.STATE_STDS,
                VisionConstants.VISION_STDS);

        photonPoseEstimator.setMultiTagFallbackStrategy(VisionConstants.POSE_FALLBACK_STRATEGY);

        odometry = new SwerveDriveOdometry(
                SwerveConstants.KINEMATICS, gyro, modulePositions);
    }

    public void update() {

        if (RobotBase.isReal() && robotEnabled)
            if (VisionConstants.VISION_POSE_ESTIMATION_ENABLED) {
                final Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimator.update();
                if (optionalEstimatedPose.isPresent()) {
                    final EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
                    poseEstimator.addVisionMeasurement(
                            estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
                }
            }

        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotationEstimate() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }
}
