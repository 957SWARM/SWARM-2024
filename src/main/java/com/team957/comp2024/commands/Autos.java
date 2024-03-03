package com.team957.comp2024.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.peripherals.LLlocalization;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class Autos {
    private class AutoPhaseFactory {
        private final Swerve swerve;
        private final IntakePivot pivot;
        private final ArrayList<ChoreoTrajectory> traj;
        private final LLlocalization localization;
        private final Supplier<Alliance> alliance;

        AutoPhaseFactory(
                Swerve swerve,
                IntakePivot pivot,
                ArrayList<ChoreoTrajectory> traj,
                LLlocalization localization,
                Supplier<Alliance> alliance) {
            this.swerve = swerve;
            this.pivot = pivot;
            this.traj = traj;
            this.localization = localization;
            this.alliance = alliance;
        }

        // TODO: safe load of potentially non-existent trajectory phases
        Command shootTrajectoryPhase(int phaseIndex, boolean resetPose) {
            return ChoreoFollowingFactory.instance
                    .getPathFollowingCommand(
                            swerve, traj.get(phaseIndex), localization, resetPose, alliance)
                    .andThen(
                            ScoringSequences.coordinatedSubwooferShot(
                                    shooter, intakePivot, intakeRoller));
        }

        Command floorTrajectoryPhase(int phaseIndex, boolean resetPose) {
            return new WaitCommand(.25)
                    .andThen(
                            ChoreoFollowingFactory.instance.getPathFollowingCommand(
                                    swerve,
                                    traj.get(phaseIndex),
                                    localization,
                                    resetPose,
                                    alliance))
                    .alongWith(ScoringSequences.coordinatedFloorIntake(intakePivot, intakeRoller));
        }

        Command stowTrajectoryPhase(int phaseIndex, boolean resetPose) {
            return ChoreoFollowingFactory.instance
                    .getPathFollowingCommand(
                            swerve, traj.get(phaseIndex), localization, resetPose, alliance)
                    .alongWith(pivot.toStow());
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
    private final Supplier<Alliance> alliance;

    private final Alert loadFailureAlert =
            new Alert("Autonomous trajectory failed to load!", AlertType.ERROR);

    public Autos(
            Swerve swerve,
            IntakePivot intakePivot,
            IntakeRoller intakeRoller,
            Shooter shooter,
            LLlocalization localization,
            Supplier<Alliance> alliance) {
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.shooter = shooter;
        this.localization = localization;
        this.alliance = alliance;
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

    private Command singleTrajectoryOnlyAuto(String trajName, boolean resetPoseToInitial) {
        var maybeTraj = safeLoadTrajectory(trajName);

        if (!maybeTraj.isPresent()) return new InstantCommand();

        return ChoreoFollowingFactory.instance.getPathFollowingCommand(
                swerve, maybeTraj.get().get(0), localization, resetPoseToInitial, alliance);
    }

    public Command justLeaveAmp() {
        return singleTrajectoryOnlyAuto("justLeaveAmp", true);
    }

    public Command justLeaveCenter() {
        return singleTrajectoryOnlyAuto("justLeaveCenter", true);
    }

    public Command justLeaveSource() {
        return singleTrajectoryOnlyAuto("justLeaveSource", true);
    }

    public Command testPath() {
        return singleTrajectoryOnlyAuto("testPath", true);
    }

    public Command fivePieceMockup() {
        return singleTrajectoryOnlyAuto("fivePieceMockup", true);
    }

    public Command fourPieceMockup() {
        return singleTrajectoryOnlyAuto("fourPieceMockup", true);
    }

    public Command threePieceMockup() {
        return singleTrajectoryOnlyAuto("threePieceMockup", true);
    }

    public Command centerTwoPiece() {
        var maybeTraj = safeLoadTrajectory("centerTwoPiece");

        if (!maybeTraj.isPresent()) return new InstantCommand();

        AutoPhaseFactory factory =
                new AutoPhaseFactory(swerve, intakePivot, maybeTraj.get(), localization, alliance);

        return ScoringSequences.coordinatedSubwooferShot(shooter, intakePivot, intakeRoller)
                .andThen(factory.floorTrajectoryPhase(0, true).withTimeout(2))
                .andThen(factory.shootTrajectoryPhase(1, false))
                .andThen(factory.stowTrajectoryPhase(2, false));
    }
}
