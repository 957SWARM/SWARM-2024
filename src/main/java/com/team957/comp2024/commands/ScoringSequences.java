package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.SequencingConstants;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import com.team957.comp2024.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoringSequences {
    public static Command coordinatedSubwooferShot(
            Shooter shooter, IntakePivot intakePivot, IntakeRoller intakeRoller) {

        return intakePivot
                .toHandoff()
                .alongWith(
                        intakeRoller
                                .slowCentering()
                                .withTimeout(SequencingConstants.CENTERING_NOTE_DURATION))
                .withTimeout(SequencingConstants.UNTIL_SHOOT_DELAY)
                .andThen(
                        intakeRoller
                                .shooterHandoff()
                                .withTimeout(SequencingConstants.SHOOT_DURATION))
                .raceWith(shooter.subwooferShot())
                .andThen(
                        intakeRoller
                                .idle()
                                .alongWith(intakePivot.toStow())
                                .alongWith(shooter.idle()));
    }

    public static Command coordinatedFloorIntake(
            IntakePivot intakePivot, IntakeRoller intakeRoller) {
        return intakePivot
                .toFloor()
                .raceWith(
                        new WaitCommand(SequencingConstants.ROLLER_DELAY)
                                .andThen(intakeRoller.floorIntakeUntilNote()))
                .andThen(intakePivot.toStow());
    }
}
