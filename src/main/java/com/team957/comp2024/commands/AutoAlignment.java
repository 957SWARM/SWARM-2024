package com.team957.comp2024.commands;

import java.util.*;

import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.LLlocalization;
import com.team957.comp2024.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignment extends Command {

    private final Swerve swerve;
    private final LLlocalization poseEstimation;

    private final Pose2d[] targetPoses = {
            VisionConstants.BLUE_AMP,
            VisionConstants.BLUE_SPEAKER_CENTER,
            VisionConstants.BLUE_SPEAKER_LEFT,
            VisionConstants.BLUE_SPEAKER_RIGHT,
            VisionConstants.RED_AMP,
            VisionConstants.RED_SPEAKER_CENTER,
            VisionConstants.RED_SPEAKER_LEFT,
            VisionConstants.RED_SPEAKER_RIGHT
    };

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public AutoAlignment(Swerve swerve, LLlocalization poseEstimation, String limelightName) {
        this.swerve = swerve;
        this.poseEstimation = poseEstimation;
        addRequirements(swerve);
    }

    public Command getAlignmentCommand() {

        return swerve.getFieldRelativeControlCommand(
                () -> {
                    return new ChassisSpeeds(
                            calculateAlignment(
                                    poseEstimation.getPoseEstimate().getX(), getNearestTarget().getX()),
                            calculateAlignment(
                                    poseEstimation.getPoseEstimate().getY(), getNearestTarget().getY()),
                            calculateRot(poseEstimation.getRotationEstimate().getRadians(),
                                    getNearestTarget().getRotation().getRadians()));
                },
                poseEstimation::getRotationEstimate);
    }

    public Pose2d getNearestTarget() {
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
