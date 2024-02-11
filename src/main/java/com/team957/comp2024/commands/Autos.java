package com.team957.comp2024.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.Constants;
import com.team957.comp2024.LLlocalization;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class Autos {
    private class AutoPhaseFactory {
        private final Swerve swerve;
        private final IntakePivot pivot;
        private final ArrayList<ChoreoTrajectory> traj;
        private final LLlocalization localization;

        AutoPhaseFactory(
                Swerve swerve,
                IntakePivot pivot,
                ArrayList<ChoreoTrajectory> traj,
                LLlocalization localization) {
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

    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Shooter shooter;
    private final LLlocalization localization;

    private final Alert loadFailureAlert =
            new Alert("Autonomous trajectory failed to load!", AlertType.ERROR);

    public Autos(
            Swerve swerve,
            IntakePivot intakePivot,
            IntakeRoller intakeRoller,
            Shooter shooter,
            LLlocalization localization) {
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.shooter = shooter;
        this.localization = localization;
    }

    private Optional<ArrayList<ChoreoTrajectory>> safeLoadTrajectory(String trajName) {
        var trajPhases = Choreo.getTrajectoryGroup(trajName);

        if (trajPhases == null) {
            loadFailureAlert.set(true);
            return Optional.empty();
        } else {
            loadFailureAlert.set(false);
            return Optional.of(trajPhases);
        }
    }

    public Command doNothingAuto() {
        return new InstantCommand();
    }

    public Command shootPreloadBumperAuto() {
        return new InstantCommand(); // shooter not implemented
    }

    public Command middleTwoPiece() {
        var phases = safeLoadTrajectory("middleTwoPiece");

        if (!phases.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, phases.get(), localization);

        return intakePivot
                .toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.lock());
    }

    public Command topNearThreePiece() {
        var phases = safeLoadTrajectory("topNearThreePiece");

        if (!phases.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, phases.get(), localization);

        return intakePivot
                .toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.floorTrajectoryPhase(2, false))
                .andThen(factory.shootTrajectoryPhase(3, false))
                .andThen(factory.lock());
    }

    public Command topCenterFourPiece() {
        var phases = safeLoadTrajectory("topCenterFourPiece");

        if (!phases.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, phases.get(), localization);

        return intakePivot
                .toHandoff()
                .withTimeout(Constants.AutoConstants.SHOOT_BUFFER_SECONDS)
                .andThen(factory.floorTrajectoryPhase(0, true))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.floorTrajectoryPhase(2, false))
                .andThen(factory.shootTrajectoryPhase(3, false))
                .andThen(factory.floorTrajectoryPhase(4, false))
                .andThen(factory.shootTrajectoryPhase(5, false))
                .andThen(factory.lock());
    }

    public Command nearFourPiece() {
        var phases = safeLoadTrajectory("nearFourPiece");

        if (!phases.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, phases.get(), localization);

        return intakePivot
                .toHandoff()
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

    public Command topFivePiece() {
        var phases = safeLoadTrajectory("topFivePiece");

        if (!phases.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, phases.get(), localization);

        return intakePivot
                .toHandoff()
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