package com.team957.comp2024.commands;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.Constants.AutoConstants;
import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.util.SwarmChoreo;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

public class ChoreoFollowing {
    public static Command getPathFollowingCommand(
            Swerve swerve, ChoreoTrajectory trajectory, Supplier<Pose2d> localization) {
        final PID xController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID yController = new PID(AutoConstants.LINEAR_PATHFINDING_GAINS, 0);
        final PID thetaController = new PID(AutoConstants.ROTATIONAL_PATHFINDING_GAINS, 0, true);

        ChoreoControlFunction controlFunction =
                SwarmChoreo.choreoSwerveController(
                        (Double measurement, Double setpoint) -> {
                            xController.setSetpoint(setpoint);
                            return xController.calculate(measurement);
                        },
                        (Double measurement, Double setpoint) -> {
                            yController.setSetpoint(setpoint);
                            return yController.calculate(measurement);
                        },
                        (Double measurement, Double setpoint) -> {
                            thetaController.setSetpoint(setpoint);
                            return thetaController.calculate(measurement);
                        });

        Timer timer = new Timer();

        return Commands.runOnce(timer::restart)
                .andThen(
                        swerve.getChassisRelativeControlCommand(
                                () ->
                                        controlFunction.apply(
                                                localization.get(),
                                                trajectory.sample(timer.get()))));
    }
}
