package com.team957.comp2024;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.comp2024.Constants.VisionConstants;
import com.team957.lib.math.filters.IntegratingFilter;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class LLlocalization {

    private final SwerveDriveKinematics kinematics;
    private final Supplier<SwerveModuleState[]> moduleStates;
    private final Supplier<SwerveModulePosition[]> modulePositions;
    private final Supplier<Rotation2d> gyro;
    private final boolean robotReal;

    private final IntegratingFilter simGyro = new IntegratingFilter(0);
    private final DeltaTimeUtil dtUtil = new DeltaTimeUtil();

    private final AprilTagFieldLayout ATLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private Pose2d visionPose2d = new Pose2d();

    private final SwerveDrivePoseEstimator poseEstimator;

    public LLlocalization(
            SwerveDriveKinematics kinematics,
            Supplier<SwerveModuleState[]> moduleStates,
            Supplier<SwerveModulePosition[]> modulePositions,
            Supplier<Rotation2d> gyro,
            boolean robotReal) {

        this.kinematics = kinematics;
        this.moduleStates = moduleStates;
        this.modulePositions = modulePositions;
        this.gyro = gyro;
        this.robotReal = robotReal;

        poseEstimator =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        gyro.get(),
                        modulePositions.get(),
                        new Pose2d(),
                        VisionConstants.STATE_STDS,
                        VisionConstants.VISION_STDS);
    }

    public void update() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        simGyro.calculate(
                SwerveConstants.KINEMATICS.toChassisSpeeds(moduleStates.get())
                        .omegaRadiansPerSecond,
                dt);

        if (robotReal) {
            estimateVisionPose("limelight");

            poseEstimator.update(gyro.get(), modulePositions.get());
        } else {
            poseEstimator.update(new Rotation2d(simGyro.getCurrentOutput()), modulePositions.get());
        }
    }

    public void estimateVisionPose(String limelightName) {

        if (VisionConstants.VISION_POSE_ESTIMATION_ENABLED) {

            double[] botpose = LimelightLib.getBotPose_wpiBlue(limelightName);

            Rotation3d rot3 =
                    new Rotation3d(
                            Units.degreesToRadians(botpose[3]),
                            Units.degreesToRadians(botpose[4]),
                            Units.degreesToRadians(botpose[5]));

            Pose3d visionPose = new Pose3d(botpose[0], botpose[1], botpose[2], rot3);

            if (visionPose != null && LimelightLib.getTV(limelightName)) {
                if (LimelightLib.getTA(limelightName) > VisionConstants.TARGET_AREA_CUTOFF) {
                    visionPose2d =
                            new Pose2d(visionPose.getTranslation().toTranslation2d(), gyro.get());
                    double timeStampSeconds =
                            Timer.getFPGATimestamp()
                                    - (LimelightLib.getLatency_Pipeline(limelightName) / 1000.0)
                                    - (LimelightLib.getLatency_Capture(limelightName) / 1000.0);

                    poseEstimator.addVisionMeasurement(visionPose2d, timeStampSeconds);
                }
            }
        }
    }

    /*
     * public double getVelocityMetersPerSecond(){
     * Swerve test;
     * return kinematics.toChassisSpeeds(test.getStates()).
     * }
     */

    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotationEstimate() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void setPose(Pose2d pose) {
        if (robotReal) {
            poseEstimator.resetPosition(gyro.get(), modulePositions.get(), pose);
        } else {
            poseEstimator.resetPosition(
                    new Rotation2d(simGyro.getCurrentOutput()), modulePositions.get(), pose);
        }
    }
}
