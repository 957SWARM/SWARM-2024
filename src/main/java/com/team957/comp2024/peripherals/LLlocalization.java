package com.team957.comp2024.peripherals;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.UI;
import com.team957.comp2024.util.LimelightHelpers;
import com.team957.lib.math.filters.IntegratingFilter;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class LLlocalization implements Logged {

    private final Supplier<SwerveModuleState[]> moduleStates;
    private final Supplier<SwerveModulePosition[]> modulePositions;
    private final Supplier<IMU> imu;
    private final Supplier<Double> fieldRotationOffset;
    private final boolean robotReal;

    private final IntegratingFilter simGyro = new IntegratingFilter(0);
    private final DeltaTimeUtil dtUtil = new DeltaTimeUtil();

    private Pose2d visionPose2d = new Pose2d();

    private final SwerveDrivePoseEstimator poseEstimator;

    AprilTagFieldLayout ATFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // PhotonCamera photonCam = new PhotonCamera(VisionConstants.PCAM_NAME);
    // PhotonPoseEstimator photonEstimator;

    // this is 100% a code smell, but something is screwy with the pose estimator
    // unless we flip
    // these
    private SwerveModulePosition[] invertDistances(SwerveModulePosition[] positions) {
        SwerveModulePosition[] flipped = positions;

        for (SwerveModulePosition position : flipped)
            position.distanceMeters = -position.distanceMeters;

        return flipped;
    }

    public LLlocalization(
            SwerveDriveKinematics kinematics,
            Supplier<SwerveModuleState[]> moduleStates,
            Supplier<SwerveModulePosition[]> modulePositions,
            Supplier<IMU> imu,
            Supplier<Double> fieldRotationOffset,
            boolean robotReal) {

        this.moduleStates = moduleStates;
        this.modulePositions = modulePositions;
        this.imu = imu;
        this.fieldRotationOffset = fieldRotationOffset;
        this.robotReal = robotReal;

        poseEstimator =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        imu.get().getCorrectedAngle(),
                        modulePositions.get(),
                        new Pose2d(),
                        VisionConstants.STATE_STDS,
                        VisionConstants.VISION_STDS);

        // photonEstimator =
        // new PhotonPoseEstimator(
        // ATFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        // photonCam,
        // VisionConstants.PCAM_TO_CENTER);

        // photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void update(boolean useVision, Supplier<IMU> imu) {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        simGyro.calculate(
                SwerveConstants.KINEMATICS.toChassisSpeeds(moduleStates.get())
                        .omegaRadiansPerSecond,
                dt);

        Rotation2d rotation;

        if (robotReal) {
            if (useVision) {
                estimateVisionPoseLL(VisionConstants.LL2_NAME, imu);
            }
            // estimateVisionPosePV();

            rotation = imu.get().getCorrectedAngle();
        } else {
            rotation = new Rotation2d(simGyro.getCurrentOutput());
        }

        poseEstimator.update(rotation, modulePositions.get());

        UI.instance.setPose(poseEstimator.getEstimatedPosition());
    }

    public void centerGyro() {
        poseEstimator.resetPosition(
                imu.get().getCorrectedAngle(),
                modulePositions.get(),
                new Pose2d(
                        getPoseEstimate().getTranslation(),
                        new Rotation2d(
                                (DriverStation.getAlliance().get() == Alliance.Red)
                                        ? Math.PI
                                        : 0)));
    }

    public void estimateVisionPoseLL(String limelightName, Supplier<IMU> imu) {

        if (VisionConstants.VISION_POSE_ESTIMATION_ENABLED
                && LimelightHelpers.getTV(limelightName)) {

            Boolean doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(
                    limelightName,
                    poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
                    0,
                    0,
                    0,
                    0,
                    0);
            LimelightHelpers.PoseEstimate mt2 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            if (Math.abs(Units.radiansToDegrees(imu.get().getAngularVelocityRadiansperSecond()))
                    > 720) // if our angular velocity is greater than 720 degrees per second, ignore
            // vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    // public void estimateVisionPosePV() {
    // if (VisionConstants.VISION_POSE_ESTIMATION_ENABLED) {
    // final Optional<EstimatedRobotPose> optionalEstimatedRobotPose =
    // photonEstimator.update();
    // if (optionalEstimatedRobotPose.isPresent()) {
    // final EstimatedRobotPose estimatedRobotPose =
    // optionalEstimatedRobotPose.get();
    // poseEstimator.addVisionMeasurement(
    // estimatedRobotPose.estimatedPose.toPose2d(),
    // estimatedRobotPose.timestampSeconds);
    // }
    // }
    // }

    @Log.NT
    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    @Log.NT
    public Rotation2d getRotationEstimate() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void setPose(Pose2d pose) {
        Rotation2d rot =
                (robotReal)
                        ? imu.get().getCorrectedAngle()
                        : new Rotation2d(simGyro.getCurrentOutput());

        poseEstimator.resetPosition(rot, modulePositions.get(), pose);
    }
}
