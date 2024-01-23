package com.team957.comp2024;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.lib.math.filters.IntegratingFilter;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Localization implements Logged {
    private final SwerveDriveOdometry odometry;

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
        odometry =
                new SwerveDriveOdometry(
                        SwerveConstants.KINEMATICS, gyro.get(), modulePositions.get());

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
            odometry.update(new Rotation2d(simGyro.getCurrentOutput()), modulePositions.get());
        } else {
            odometry.update(gyro.get(), modulePositions.get());
        }

        UI.instance.setPose(getPoseEstimate());
    }

    @Log.NT
    public Pose2d getPoseEstimate() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getRotationEstimate() {
        return odometry.getPoseMeters().getRotation();
    }

    public void setPose(Pose2d pose) {
        if (fakeGyro) {
            odometry.resetPosition(
                    new Rotation2d(simGyro.getCurrentOutput()), modulePositions.get(), pose);
        } else {
            odometry.resetPosition(gyro.get(), modulePositions.get(), pose);
        }
    }
}
