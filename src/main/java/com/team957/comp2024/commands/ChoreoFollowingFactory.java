package com.team957.comp2024.commands;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.AutoConstants;
import com.team957.comp2024.UI;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import monologue.Annotations.IgnoreLogged;
import monologue.Logged;

public class ChoreoFollowingFactory implements Logged {
    // being instance is slightly janky but needed to have a monologue context, but singleton works

    @IgnoreLogged
    public static final ChoreoFollowingFactory instance = new ChoreoFollowingFactory();

    private ChoreoFollowingFactory() {}

    private ChoreoControlFunction alternateControlFunction(PID x, PID y, PID theta) {
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double tFF = referenceState.angularVelocity;

            this.log("referenceChassisSpeeds", new ChassisSpeeds(xFF, yFF, tFF));

            x.setSetpoint(referenceState.x);
            double xFB = x.calculate(pose.getX());

            y.setSetpoint(referenceState.y);
            double yFB = y.calculate(pose.getY());

            theta.setSetpoint(referenceState.heading);
            double tFB = theta.calculate(pose.getRotation().getRadians());

            return new ChassisSpeeds(xFF + xFB, yFF + yFB, tFF + tFB);
        };
    }

    public Command getPathFollowingCommand(
            Swerve swerve,
            ChoreoTrajectory trajectory,
            LLlocalization localization,
            boolean resetPoseToInitial,
            Supplier<Alliance> alliance) {

        final PID xController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID yController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID thetaController = new PID(AutoConstants.ROTATIONAL_PATHFINDING_GAINS, 0, true);

        final ChoreoControlFunction controlFunction =
                alternateControlFunction(xController, yController, thetaController);

        final Timer timer = new Timer();

        final ChoreoTrajectory flipped = trajectory.flipped();
        final Supplier<ChoreoTrajectory> getMirroredPath =
                () -> {
                    return (alliance.get() == Alliance.Blue) ? trajectory : flipped;
                };

        return Commands.runOnce(
                        () -> {
                            timer.restart();
                            if (resetPoseToInitial)
                                localization.setPose(getMirroredPath.get().getInitialPose());
                        })
                // .andThen(Commands.runOnce((this.log("trajectory",trajectory))))
                // choreotrajectory not supported in monologue :(
                .andThen(
                        swerve.getChassisRelativeControlCommand(
                                        () ->
                                                controlFunction.apply(
                                                        localization.getPoseEstimate(),
                                                        getMirroredPath.get().sample(timer.get())))
                                .alongWith(
                                        Commands.run(
                                                () -> {
                                                    Pose2d pose =
                                                            getMirroredPath
                                                                    .get()
                                                                    .sample(timer.get())
                                                                    .getPose();

                                                    this.log("poseSetpoint", pose);

                                                    UI.instance.setSetpointPose(pose);
                                                })))
                .until(
                        () ->
                                getMirroredPath.get().getTotalTime()
                                        < timer.get()
                                                - Constants.AutoConstants
                                                        .PROFILE_OVERRUN_TOLERANCE_SECONDS)
                .withName("choreoFollowing");
    }
}
