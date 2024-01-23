package com.team957.comp2024.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.Constants;
import com.team957.comp2024.Localization;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;

public class Autos {
    private static class AutoPhaseFactory {
        private final Swerve swerve;
        private final IntakePivot pivot;
        private final ArrayList<ChoreoTrajectory> traj;
        private final Localization localization;

        AutoPhaseFactory(
                Swerve swerve,
                IntakePivot pivot,
                ArrayList<ChoreoTrajectory> traj,
                Localization localization) {
            this.swerve = swerve;
            this.pivot = pivot;
            this.traj = traj;
            this.localization = localization;
        }

        Command shootTrajectoryPhase(int phaseIndex, double pivotDelaySeconds, boolean resetPose) {
            return ChoreoFollowingFactory.instance
                    .getPathFollowingCommand(swerve, traj.get(phaseIndex), localization, resetPose)
                    .alongWith(new WaitCommand(pivotDelaySeconds).andThen(pivot.toHandoff()))
                    .withTimeout(
                            traj.get(phaseIndex).getTotalTime()
                                    + Constants.AutoConstants.SHOOT_BUFFER_SECONDS);
        }

        Command shootTrajectoryPhase(int phaseIndex, boolean resetPose) {
            return shootTrajectoryPhase(
                    phaseIndex, Constants.AutoConstants.DEFAULT_PIVOT_DELAY_SECONDS, resetPose);
        }

        Command floorTrajectoryPhase(int phaseIndex, double pivotDelaySeconds, boolean resetPose) {
            return ChoreoFollowingFactory.instance
                    .getPathFollowingCommand(swerve, traj.get(phaseIndex), localization, resetPose)
                    .alongWith(new WaitCommand(pivotDelaySeconds).andThen(pivot.toFloor()))
                    .withTimeout(
                            traj.get(phaseIndex).getTotalTime()
                                    + Constants.AutoConstants.INTAKE_BUFFER_SECONDS);
        }

        Command floorTrajectoryPhase(int phaseIndex, boolean resetPose) {
            return floorTrajectoryPhase(
                    phaseIndex, Constants.AutoConstants.DEFAULT_PIVOT_DELAY_SECONDS, resetPose);
        }

        Command stowTrajectoryPhase(int phaseIndex, double pivotDelaySeconds, boolean resetPose) {
            return ChoreoFollowingFactory.instance
                    .getPathFollowingCommand(swerve, traj.get(phaseIndex), localization, resetPose)
                    .alongWith(new WaitCommand(pivotDelaySeconds).andThen(pivot.toStow()))
                    .withTimeout(traj.get(phaseIndex).getTotalTime());
        }

        Command stowTrajectoryPhase(int phaseIndex, boolean resetPose) {
            return stowTrajectoryPhase(
                    phaseIndex, Constants.AutoConstants.DEFAULT_PIVOT_DELAY_SECONDS, resetPose);
        }

        Command lock() {
            return swerve.lockDrivetrain().alongWith(pivot.toStow());
        }
    }

    public static Command doNothingAuto() {
        return new InstantCommand();
    }

    public static Command shootPreloadBumperAuto() {
        return new InstantCommand(); // shooter not implemented
    }

    public static Command middleTwoPiece(
            Swerve swerve, IntakePivot pivot, Localization localization) {

        AutoPhaseFactory factory =
                new AutoPhaseFactory(
                        swerve, pivot, Choreo.getTrajectoryGroup("middleTwoPiece"), localization);

        return pivot.toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.lock());
    }

    public static Command topNearThreePiece(
            Swerve swerve, IntakePivot pivot, Localization localization) {
        AutoPhaseFactory factory =
                new AutoPhaseFactory(
                        swerve,
                        pivot,
                        Choreo.getTrajectoryGroup("topNearThreePiece"),
                        localization);

        return pivot.toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.floorTrajectoryPhase(2, false))
                .andThen(factory.shootTrajectoryPhase(3, false))
                .andThen(factory.lock());
    }

    public static Command topCenterFourPiece(
            Swerve swerve, IntakePivot pivot, Localization localization) {
        AutoPhaseFactory factory =
                new AutoPhaseFactory(
                        swerve,
                        pivot,
                        Choreo.getTrajectoryGroup("topCenterFourPiece"),
                        localization);

        return pivot.toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.floorTrajectoryPhase(2, false))
                .andThen(factory.shootTrajectoryPhase(3, false))
                .andThen(factory.floorTrajectoryPhase(4, false))
                .andThen(factory.shootTrajectoryPhase(5, false))
                .andThen(factory.lock());
    }

    public static Command nearFourPiece(
            Swerve swerve, IntakePivot pivot, Localization localization) {
        AutoPhaseFactory factory =
                new AutoPhaseFactory(
                        swerve, pivot, Choreo.getTrajectoryGroup("nearFourPiece"), localization);

        return pivot.toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, 0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.floorTrajectoryPhase(2, 0, false))
                .andThen(factory.shootTrajectoryPhase(3, false))
                .andThen(factory.floorTrajectoryPhase(4, 0, false))
                .andThen(factory.shootTrajectoryPhase(5, false))
                .andThen(factory.stowTrajectoryPhase(6, false))
                .andThen(factory.lock());
    }

    public static Command topFivePiece(
            Swerve swerve, IntakePivot pivot, Localization localization) {
        AutoPhaseFactory factory =
                new AutoPhaseFactory(
                        swerve, pivot, Choreo.getTrajectoryGroup("topFivePiece"), localization);

        return pivot.toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, 1, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.floorTrajectoryPhase(2, 0, false))
                .andThen(factory.shootTrajectoryPhase(3, false))
                .andThen(factory.floorTrajectoryPhase(4, 0, false))
                .andThen(factory.shootTrajectoryPhase(5, false))
                .andThen(factory.floorTrajectoryPhase(6, 0, false))
                .andThen(factory.shootTrajectoryPhase(7, false))
                .andThen(factory.stowTrajectoryPhase(8, false))
                .andThen(factory.lock());
    }
}
