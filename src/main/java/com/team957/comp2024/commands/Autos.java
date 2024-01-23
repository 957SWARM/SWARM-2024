package com.team957.comp2024.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team957.comp2024.Localization;
import com.team957.comp2024.subsystems.intake.IntakePivot;
import com.team957.comp2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;

public class Autos {
    public static Command doNothingAuto() {
        return new InstantCommand();
    }

    public static Command shootPreloadBumperAuto() {
        return new InstantCommand(); // shooter not implemented
    }

    public static Command middleTwoPiece(
            Swerve swerve, IntakePivot pivot, Localization localization) {
        ArrayList<ChoreoTrajectory> traj = Choreo.getTrajectoryGroup("middleTwoPiece");

        return pivot.toHandoff()
                .withTimeout(1)
                .andThen(
                        ChoreoFollowingFactory.instance
                                .getPathFollowingCommand(swerve, traj.get(0), localization, true)
                                .alongWith(pivot.toFloor()))
                .withTimeout(2)
                .andThen(
                        ChoreoFollowingFactory.instance
                                .getPathFollowingCommand(swerve, traj.get(1), localization, false)
                                .alongWith(pivot.toHandoff()));
    }
}
