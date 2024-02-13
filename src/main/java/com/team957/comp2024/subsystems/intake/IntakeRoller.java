package com.team957.comp2024.subsystems.intake;

import com.team957.comp2024.Constants.IntakeRollerConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class IntakeRoller implements Subsystem, Logged {
    private final Debouncer noteIsPresentDebouncer =
            new Debouncer(IntakeRollerConstants.TOF_NOTE_PRESENT_DEBOUNCE_SECONDS);

    private boolean debouncedNotePresent = false;

    // sets the voltage to the roller motor
    public abstract void setRollerVoltage(double voltage);

    // returns the current voltage of the roller motor
    @Log.NT
    public abstract double getAppliedVoltage();

    // returns the current amps of the roller motor
    @Log.NT
    public abstract double getRollerAmps();

    @Log.NT
    public abstract boolean noteIsPresent();

    @Log.NT
    public boolean debouncedNoteIsPresent() {
        return debouncedNotePresent;
    }

    public abstract void setSimulationNoteIsPresentMock(Supplier<Boolean> mock);

    protected IntakeRoller() {
        register();
    }

    @Override
    public void periodic() {
        debouncedNotePresent = noteIsPresentDebouncer.calculate(noteIsPresent());
    }

    public static IntakeRoller getIntakeRoller(boolean isReal) {
        return (isReal) ? new IntakeRollerHW() : new IntakeRollerSim();
    }

    // intakes notes at constant voltage
    public Command floorIntake() {
        return run(() -> setRollerVoltage(IntakeRollerConstants.FLOOR_INTAKE_VOLTAGE));
    }

    public Command floorIntakeUntilNote() {
        return floorIntake().until(this::debouncedNoteIsPresent).andThen(idle());
    }

    // puking is given higher priority scheduling over intaking
    public Command eject() {
        return run(() -> setRollerVoltage(IntakeRollerConstants.EJECT_VOLTAGE));
    }

    public Command ejectUntilNoteGone() {
        return eject().until(() -> !debouncedNoteIsPresent()).andThen(idle());
    }

    // stops the intake roller
    public Command idle() {
        return run(() -> setRollerVoltage(0));
    }

    public Command shooterHandoff() {
        return run(() -> setRollerVoltage(IntakeRollerConstants.SHOOTER_HANDOFF_VOLTAGE));
    }

    public Command shooterHandoffUntilNoteGone() {
        return shooterHandoff().until(() -> !debouncedNoteIsPresent()).andThen(idle());
    }

    public Command ampShot() {
        return run(() -> setRollerVoltage(IntakeRollerConstants.AMP_SHOT_VOLTAGE));
    }

    public Command ampShotUntilNoteGone() {
        return ampShot().until(() -> !debouncedNoteIsPresent()).andThen(idle());
    }
}
