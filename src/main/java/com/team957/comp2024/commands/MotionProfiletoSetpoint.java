package com.team957.comp2024.commands;

import com.team957.comp2024.Constants;
import com.team957.comp2024.subsystems.intake.Pivot;
import com.team957.lib.math.UtilityMath;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import monologue.Logged;

public class MotionProfiletoSetpoint extends Command implements Logged {
    double targetSetpoint;
    Pivot intakePivot;
    TrapezoidProfile motionProfile;
    TrapezoidProfile.State start;
    TrapezoidProfile.State end;
    Timer timer = new Timer();

    public MotionProfiletoSetpoint(double targetSetpoint, Pivot intakePivot) {
        addRequirements(intakePivot);
        this.targetSetpoint = targetSetpoint;
        this.intakePivot = intakePivot;
        motionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.6, 0.8));
        // this +.15 may cause the stuttering effect because robot thinks its
        start = new TrapezoidProfile.State(this.intakePivot.getOffsetRev() + .15, 0);
        end = new TrapezoidProfile.State(targetSetpoint, 0);
    }

    public double getMotionProfileOutput() {
        return motionProfile.calculate(timer.get(), start, end).position;
    }

    @Log.NT
    public double getFeedForward() {
        return Constants.PivotConstants.KG
                * Math.cos(
                        (intakePivot.getOffsetRev() - Constants.PivotConstants.OFFSET_TO_STRAIGHT)
                                * 2
                                * Math.PI);
    }

    public double normalizeRev(double revs) {
        return UtilityMath.normalizeAngleRadians(revs * 2 * Math.PI) / (2 * Math.PI);
    }

    public void execute() {
        System.out.println(targetSetpoint);
        // targetSetpoint + Constants.PivotConstants.OFFSET_REV
        intakePivot.setSetpoint(
                getMotionProfileOutput() + Constants.PivotConstants.OFFSET_REV, getFeedForward());
    }

    public void initialize() {
        timer.restart();
    }
}
