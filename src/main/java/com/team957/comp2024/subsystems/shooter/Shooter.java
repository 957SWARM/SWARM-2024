package com.team957.comp2024.subsystems.shooter;

import com.team957.comp2024.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Shooter implements Subsystem, Logged {

    public void setShooterVoltage(double voltage) {
        setLeftVoltage(voltage);
        setRightVoltage(voltage);
    }

    public abstract void setLeftVoltage(double voltage);

    public abstract void setRightVoltage(double voltage);

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

    @Log.NT
    public double getVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2;
    }

    @Log.NT
    public abstract double getRightVelocity();

    @Log.NT
    public abstract double getLeftVelocity();

    protected Shooter() {
        register();
    }

    @Override
    public void periodic() {
        Command activeCommand = getCurrentCommand();

        if (activeCommand != null) log("activeCommand", activeCommand.getName());
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

    public Command subwooferShot() {
        return defaultShooterControlCommand(
                        () -> Constants.ShooterConstants.SUBWOOFER_CONTROL_EFFORT_VOLTS)
                .withName("subwooferShot");
    }

    public Command idle() {
        return defaultShooterControlCommand(
                        () -> Constants.ShooterConstants.IDLE_CONTROL_EFFORT_VOLTS)
                .withName("idle");
    }

    public Command off() {
        return defaultShooterControlCommand(() -> 0.0).withName("noVoltage");
    }
}
