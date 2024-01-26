package com.team957.comp2024;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.lib.math.filters.IntegratingFilter;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;

public class Localization implements Logged {
    private final SwerveDrivePoseEstimator estimator;

    private final Supplier<SwerveModuleState[]> moduleStates;
    private final Supplier<SwerveModulePosition[]> modulePositions;
    private final Supplier<Rotation2d> gyro;

    private final boolean fakeGyro;

    private final IntegratingFilter simGyro = new IntegratingFilter(0);
    private final DeltaTimeUtil dtUtil = new DeltaTimeUtil();

    public Localization(
            Supplier<SwerveModuleState[]> moduleStates,
            Supplier<SwerveModulePosition[]> modulePositions,
            Supplier<Rotation2d> gyro,
            boolean fakeGyro) {
        estimator =
                new SwerveDrivePoseEstimator(
                        SwerveConstants.KINEMATICS,
                        gyro.get(),
                        modulePositions.get(),
                        new Pose2d());

        this.moduleStates = moduleStates;
        this.modulePositions = modulePositions;
        this.gyro = gyro;

        this.fakeGyro = fakeGyro;
    }

    public void update() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        simGyro.calculate(
                SwerveConstants.KINEMATICS.toChassisSpeeds(moduleStates.get())
                        .omegaRadiansPerSecond,
                dt);

        if (fakeGyro) {
            estimator.update(new Rotation2d(simGyro.getCurrentOutput()), modulePositions.get());
        } else {
            estimator.update(gyro.get(), modulePositions.get());
        }

        Robot.ui.setPose(getPoseEstimate());
    }

    @Log.NT
    public Pose2d getPoseEstimate() {
        return estimator.getEstimatedPosition();
    }

    public Rotation2d getRotationEstimate() {
        return estimator.getEstimatedPosition().getRotation();
    }

    public void setPose(Pose2d pose) {
        if (fakeGyro) {
            estimator.resetPosition(
                    new Rotation2d(simGyro.getCurrentOutput()), modulePositions.get(), pose);
        } else {
            estimator.resetPosition(gyro.get(), modulePositions.get(), pose);
        }
    }

    public void addVisionMeasurement(EstimatedRobotPose estimate) {
        estimator.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
    }
}
