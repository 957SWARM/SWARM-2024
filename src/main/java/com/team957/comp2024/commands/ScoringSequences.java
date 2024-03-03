package com.team957.comp2024.commands;

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
                .alongWith(intakeRoller.slowCentering().withTimeout(.05))
                .withTimeout(.6)
                .andThen(intakeRoller.shooterHandoff().withTimeout(.5))
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
                .raceWith(new WaitCommand(.1).andThen(intakeRoller.floorIntakeUntilNote()))
                .andThen(intakePivot.toStow());
    }
}
