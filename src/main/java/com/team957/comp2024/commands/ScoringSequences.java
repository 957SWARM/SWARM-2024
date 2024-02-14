package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.IntakeRollerConstants;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoringSequences {
    public static Command coordinatedSubwooferShot(
            Shooter shooter, IntakePivot intakePivot, IntakeRoller intakeRoller) {
        return shooter.subwooferShot()
                .raceWith(
                        intakePivot
                                .toHandoff()
                                .andThen(intakeRoller.shooterHandoffUntilNoteGone())
                                .alongWith(
                                        new WaitCommand(
                                                        IntakeRollerConstants
                                                                .SIM_MOCK_OUTTAKE_DELAY_SECONDS)
                                                .andThen(
                                                        () ->
                                                                intakeRoller
                                                                        .setSimulationNoteIsPresentMock(
                                                                                false))))
                .andThen(intakePivot.toStow());
    }

    public static Command coordinatedAmpShot(IntakePivot intakePivot, IntakeRoller intakeRoller) {
        return intakePivot
                .toAmp()
                .andThen(
                        intakeRoller
                                .ampShotUntilNoteGone()
                                .alongWith(
                                        new WaitCommand(
                                                        IntakeRollerConstants
                                                                .SIM_MOCK_OUTTAKE_DELAY_SECONDS)
                                                .andThen(
                                                        () ->
                                                                intakeRoller
                                                                        .setSimulationNoteIsPresentMock(
                                                                                false))))
                .andThen(intakePivot.toStow());
    }

    public static Command coordinatedFloorIntake(
            IntakePivot intakePivot, IntakeRoller intakeRoller) {
        return intakePivot
                .toFloor()
                .andThen(
                        intakeRoller
                                .floorIntakeUntilNote()
                                .alongWith(
                                        new WaitCommand(
                                                        IntakeRollerConstants
                                                                .SIM_MOCK_INTAKE_DELAY_SECONDS)
                                                .andThen(
                                                        () ->
                                                                intakeRoller
                                                                        .setSimulationNoteIsPresentMock(
                                                                                true))))
                .andThen(intakePivot.toStow());
    }
}
