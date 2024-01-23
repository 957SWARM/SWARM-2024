package com.team957.comp2024.commands;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.Constants.AutoConstants;
import com.team957.comp2024.Localization;
import com.team957.comp2024.UI;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            Localization localization,
            boolean resetPoseToInitial) {
        final PID xController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID yController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID thetaController = new PID(AutoConstants.ROTATIONAL_PATHFINDING_GAINS, 0, true);

        ChoreoControlFunction controlFunction =
                alternateControlFunction(xController, yController, thetaController);

        Timer timer = new Timer();

        return Commands.runOnce(
                        () -> {
                            timer.restart();
                            if (resetPoseToInitial)
                                localization.setPose(trajectory.getInitialPose());
                        })
                // .andThen(Commands.runOnce((this.log("trajectory",trajectory))))
                // choreotrajectory not supported in monologue :(
                .andThen(
                        swerve.getChassisRelativeControlCommand(
                                        () ->
                                                controlFunction.apply(
                                                        localization.getPoseEstimate(),
                                                        trajectory.sample(timer.get())))
                                .alongWith(
                                        Commands.run(
                                                () -> {
                                                    Pose2d pose =
                                                            trajectory
                                                                    .sample(timer.get())
                                                                    .getPose();

                                                    this.log("poseSetpoint", pose);

                                                    UI.instance.setSetpointPose(pose);
                                                })));
    }
}
