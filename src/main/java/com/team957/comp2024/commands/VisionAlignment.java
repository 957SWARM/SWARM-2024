package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.*;
import java.util.function.Supplier;

public class VisionAlignment extends Command {

    private final Swerve swerve;
    private final LLlocalization poseEstimation;

    private double swerveX;
    private double swerveY;
    private double fieldRelRotationOffset;

    Pose2d amp = new Pose2d();
    Pose2d subwooferCenter = new Pose2d();
    Pose2d subwooferLeft = new Pose2d();
    Pose2d subwooferRight = new Pose2d();
    Pose2d speaker = new Pose2d();

    private Pose2d[] targetPoses = {amp, subwooferCenter, subwooferLeft, subwooferRight, speaker};

    private final Pose2d[] blueTargetPoses = {
        VisionConstants.BLUE_AMP,
        VisionConstants.BLUE_SUBWOOFER_CENTER,
        VisionConstants.BLUE_SUBWOOFER_LEFT,
        VisionConstants.BLUE_SUBWOOFER_RIGHT,
        VisionConstants.BLUE_SPEAKER
    };

    private final Pose2d[] redTargetPoses = {
        VisionConstants.RED_AMP,
        VisionConstants.RED_SUBWOOFER_CENTER,
        VisionConstants.RED_SUBWOOFER_LEFT,
        VisionConstants.RED_SUBWOOFER_RIGHT,
        VisionConstants.RED_SPEAKER
    };

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void initialize() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetPoses[0] = redTargetPoses[0]; // SHOULD PROBABLY BE A FOR LOOP
            targetPoses[1] = redTargetPoses[1];
            targetPoses[2] = redTargetPoses[2];
            targetPoses[3] = redTargetPoses[3];
            targetPoses[4] = redTargetPoses[4];
        } else {
            targetPoses[0] = blueTargetPoses[0];
            targetPoses[1] = blueTargetPoses[1];
            targetPoses[2] = blueTargetPoses[2];
            targetPoses[3] = blueTargetPoses[3];
            targetPoses[4] = blueTargetPoses[4];
        }

        amp = targetPoses[0];
        subwooferCenter = targetPoses[1];
        subwooferLeft = targetPoses[2];
        subwooferRight = targetPoses[3];
        speaker = targetPoses[4];
    }

    public VisionAlignment(
            Swerve swerve,
            Supplier<Double> swerveX,
            Supplier<Double> swerveY,
            LLlocalization poseEstimation) {
        this.swerve = swerve;
        this.swerveX = swerveX.get();
        this.swerveY = swerveY.get();
        this.poseEstimation = poseEstimation;
        addRequirements(swerve);
    }

    public Command getSpeakerTrackCommand() {
        return swerve.getFieldRelativeControlCommand(
                () -> {
                    return new ChassisSpeeds(
                            swerveX,
                            swerveY,
                            calculateRot(
                                    poseEstimation.getRotationEstimate().getDegrees(),
                                    getSpeakerAngle()));
                },
                poseEstimation::getRotationEstimate);
    }

    public double getSpeakerAngle() { // TODO: TEST BY PRINTING GETSPEAKERANGLE
        Pose2d pointToFace = new Pose2d(0.21, 5.5, new Rotation2d(Math.PI));
        Pose2d difference =
                new Pose2d(
                        pointToFace
                                .getTranslation()
                                .minus(poseEstimation.getPoseEstimate().getTranslation()),
                        pointToFace
                                .getRotation()
                                .minus(poseEstimation.getRotationEstimate())
                                .minus(new Rotation2d(Math.PI)));

        // double c =
        //         Math.sqrt(
        //                 (difference.getY() * difference.getY())
        //                         + (difference.getX() * difference.getX()));
        // double rotationNeeded = Math.asin(difference.getY() / c);
        // return Units.radiansToDegrees(rotationNeeded);

        return difference.getRotation().getRadians();
    }

    public Command getTargetAlignmentCommand() {
        if (getNearestTarget() != null) {
            return swerve.getFieldRelativeControlCommand(
                    () -> {
                        return new ChassisSpeeds(
                                calculateAlignment(
                                        poseEstimation.getPoseEstimate().getX(),
                                        getNearestTarget().getX()),
                                calculateAlignment(
                                        poseEstimation.getPoseEstimate().getY(),
                                        getNearestTarget().getY()),
                                calculateRot(
                                        poseEstimation
                                                .getRotationEstimate()
                                                .getRadians(), // MIGHT NEED DEGREES
                                        getNearestTarget().getRotation().getRadians()));
                    },
                    () ->
                            poseEstimation
                                    .getRotationEstimate()
                                    .minus(new Rotation2d(fieldRelRotationOffset)));
        } else {
            return null;
        }
    }

    public Pose2d getNearestTarget() {
        // SHOULD PROBABLY HAVE A MAX DISTANCE
        return poseEstimation.getPoseEstimate().nearest(Arrays.asList(targetPoses));
    }

    public double calculatePLoop(
            double current,
            double target,
            double kp,
            double minCommand,
            double stopThreshold,
            double minCommandThreshold,
            double maxSpeed) {
        double diff = current - target;
        if (Math.abs(diff) > stopThreshold) {
            if (diff > 0 && diff < minCommandThreshold) {
                return clamp(-minCommand, -maxSpeed, maxSpeed);
            } else if (diff < 0 && diff > -minCommandThreshold) {
                return clamp(minCommand, -maxSpeed, maxSpeed);
            } else {
                return clamp(kp * diff, -maxSpeed, maxSpeed);
            }
        }
        return 0;
    }

    public double calculateAlignment(double currentX, double targetX) {
        return calculatePLoop(
                currentX,
                targetX,
                VisionConstants.ALIGNING_KP,
                VisionConstants.ALIGNING_MIN_COMMAND,
                VisionConstants.ALIGNING_STOP_THRESHOLD,
                VisionConstants.ALINGING_MIN_COMMAND_TRESHOLD,
                VisionConstants.ALIGNING_MAX_SPEED);
    }

    public double calculateRot(double currentRot, double targetRot) {
        return calculatePLoop(
                currentRot,
                targetRot,
                // MIGHT HAVE TO CHANGE TO CUSTOM VALUES INSTEAD OF TRACKING ONES
                VisionConstants.TRACKING_KP,
                VisionConstants.TRACKING_MIN_COMMAND,
                VisionConstants.TRACKING_STOP_THRESHOLD,
                VisionConstants.TRACKING_MIN_COMMAND_TRESHOLD,
                VisionConstants.TRACKING_MAX_SPEED);
    }
}
