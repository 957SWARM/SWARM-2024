package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.VisionConstants;
import com.team957.comp2024.LimelightLib;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class NoteTargetingCommand extends Command {

    private final Swerve swerve;
    private final String limelightName;

    public double map(
            double input,
            double inputStart,
            double inputEnd,
            double outputStart,
            double outputEnd) {
        double slope = 1 * (outputEnd - outputStart) / (inputEnd - inputStart);
        return outputStart + slope * (input - inputStart);
    }

    public NoteTargetingCommand(Swerve swerve, String limelightName) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        addRequirements(swerve);
    }

    public void execute() {}

    public void trackNote() {}

    public Pose2d getNotePose2d() {

        double groundDistance = getNoteGroundDistance();
        double tx = LimelightLib.getTX(limelightName);
        double targetYLL;
        double targetXLL;
        if (tx > 0) {
            targetYLL = -(groundDistance * Math.sin(Units.degreesToRadians(Math.abs(tx))));
        } else {
            targetYLL = (groundDistance * Math.sin(Units.degreesToRadians(Math.abs(tx))));
        }
        targetXLL =
                Math.sqrt(
                        (groundDistance * groundDistance)
                                - (Math.abs(targetYLL) * Math.abs(targetYLL)));

        double targetXRobot =
                targetXLL + Units.metersToInches(VisionConstants.LL1_TO_CENTER.getX());
        double targetYRobot =
                targetYLL + Units.metersToInches(VisionConstants.LL1_TO_CENTER.getY());

        Pose2d targetPose =
                new Pose2d(new Translation2d(targetXRobot, targetYRobot), new Rotation2d());

        return targetPose;
    }

    public double getNoteDistance() {
        double tx = LimelightLib.getTX(limelightName);
        double thor = LimelightLib.getTHOR(limelightName);
        double txp =
                map(
                        tx,
                        -(VisionConstants.LL_FOV_DEGREES / 2),
                        (VisionConstants.LL_FOV_DEGREES / 2),
                        0,
                        VisionConstants.LL_FOV_PIXELS);
        double txpOffset = txp + (thor / 2);
        double txOffset =
                map(
                        txpOffset,
                        0,
                        VisionConstants.LL_FOV_PIXELS,
                        -(VisionConstants.LL_FOV_DEGREES / 2),
                        (VisionConstants.LL_FOV_DEGREES / 2));
        double angle = Math.abs(txOffset - tx);

        double c = (VisionConstants.NOTE_WIDTH / 2) / Math.sin(Units.degreesToRadians(angle));

        double distance =
                Math.sqrt(
                        (c * c)
                                - ((VisionConstants.NOTE_WIDTH / 2)
                                        * (VisionConstants.NOTE_WIDTH / 2)));

        // super secret krabby patty distance formula 3
        double distanceCorrected = (distance - ((1.15 * (distance - 5)) - distance)) * .95;

        if (LimelightLib.getTV(limelightName)) {
            return distanceCorrected;
        } else {
            return 0;
        }
    }

    public double getNoteGroundDistance() {
        double distance = getNoteDistance();
        double groundDistance =
                Math.sqrt(
                        (distance * distance)
                                - (VisionConstants.LL1_TO_CENTER.getZ()
                                        * VisionConstants.LL1_TO_CENTER.getZ()));
        return groundDistance;
    }
}
