package com.team957.comp2024.commands;

import com.team957.comp2024.LLlocalization;
import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.subsystems.swerve.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignment extends Command {

    private final Swerve swerve;
    private final LLlocalization poseEstimation;
    private final String limelightName;

    private final Pose2d blueAmp = VisionConstants.BLUE_AMP;
    private final Pose2d redAmp = VisionConstants.BLUE_AMP;

    public AutoAlignment(Swerve swerve, LLlocalization poseEstimation, String limelightName) {
        this.swerve = swerve;
        this.poseEstimation = poseEstimation;
        this.limelightName = limelightName;
        addRequirements(swerve);
    }

    public Command getAlignmentCommand() {

        return swerve.getFieldRelativeControlCommand(
                () -> {
                    return new ChassisSpeeds(
                            null, null, null);
                },
                poseEstimation::getRotationEstimate);
    }

    public double calculateX(double currentX, double targetX) {
        double xDiff = currentX - targetX;
        double kp = VisionConstants.ALIGNING_KP;
        double minCommand = VisionConstants.ALIGNING_MIN_COMMAND;
        if (Math.abs(xDiff) > VisionConstants.ALIGNING_STOP_THRESHOLD) {
            if (xDiff > 0 && xDiff < VisionConstants.ALINGING_MIN_COMMAND_TRESHOLD) {
                return -minCommand;
            } else if (xDiff < 0 && xDiff > -VisionConstants.ALINGING_MIN_COMMAND_TRESHOLD) {
                return minCommand;
            } else {
                return kp * xDiff;
            }
        }
        return 0;
    }

    public double calculateY(double currentY, double targetY) {
        double xDiff = currentY - targetY;
        double kp = VisionConstants.ALIGNING_KP;
        double minCommand = VisionConstants.ALIGNING_MIN_COMMAND;
        if (Math.abs(xDiff) > VisionConstants.ALIGNING_STOP_THRESHOLD) {
            if (xDiff > 0 && xDiff < VisionConstants.ALINGING_MIN_COMMAND_TRESHOLD) {
                return -minCommand;
            } else if (xDiff < 0 && xDiff > -VisionConstants.ALINGING_MIN_COMMAND_TRESHOLD) {
                return minCommand;
            } else {
                return kp * xDiff;
            }
        }
        return 0;
    }

}
