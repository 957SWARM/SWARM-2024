package com.team957.comp2024.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Shooter implements Subsystem, Logged {

    // sets the voltage to the shooter. Motors should follow each other!
    public abstract void setShooterVoltage(double voltage);

    // returns the current voltage of the left motor
    @Log.NT
    public abstract double getLeftMotorVoltage();

    // returns the current voltage of the right motor
    @Log.NT
    public abstract double getRightMotorVoltage();

    // returns the current amps of the left motor
    @Log.NT
    public abstract double getLeftMotorAmps();

    // returns the current amps of the right motor
    @Log.NT
    public abstract double getRightMotorAmps();

    // returns the current RPM of the shooter
    @Log.NT
    public abstract double getVelocity();

    protected Shooter() {

        register();
    }

    public static Shooter getShooter(boolean isReal) {
        return (isReal) ? new ShooterHW() : new ShooterSim();
    }

    public Command defaultShooterControlCommand(Supplier<Double> voltage) {
        return run(
                () -> {
                    setShooterVoltage(voltage.get());
                });
    }

}
