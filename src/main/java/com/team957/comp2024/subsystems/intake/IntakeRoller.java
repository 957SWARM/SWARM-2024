package com.team957.comp2024.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.Supplier;

import com.team957.comp2024.Constants.IntakeRollerConstants;

import monologue.Annotations.Log;
import monologue.Logged;

public abstract class IntakeRoller implements Subsystem, Logged {

    // sets the voltage to the roller motor
    public abstract void setRollerVoltage(double voltage);

    // returns the current voltage of the roller motor
    @Log.NT
    public abstract double getRollerVoltage();

    // returns the current amps of the roller motor
    @Log.NT
    public abstract double getRollerAmps();

    protected IntakeRoller() {

        register();
    }

    public static IntakeRoller getIntakeRoller(boolean isReal) {
        return (isReal) ? new IntakeRollerHW() : new IntakeRollerSim();
    }

    // intakes notes at constant voltage
    public Command intakeNoteCommand() {
        return run(
                () -> {
                    setRollerVoltage(IntakeRollerConstants.INTAKE_VOLTAGE);
                });
    }

    // puking is given higher priority scheduling over intaking
    public Command pukeNoteCommand(){
        return run(
            () -> {
                setRollerVoltage(IntakeRollerConstants.PUKE_VOLTAGE);
            }
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    // stops the intake roller
    public Command idleCommand(){
        return run(
            () -> {
                setRollerVoltage(0);
            }
        );
    }
}