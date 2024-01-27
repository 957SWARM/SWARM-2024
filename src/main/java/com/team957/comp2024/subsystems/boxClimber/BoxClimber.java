package com.team957.comp2024.subsystems.boxClimber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public abstract class BoxClimber implements Subsystem, Logged{
    public abstract void setMotorVoltage();

    public abstract double getMotorVoltage();

    public abstract double getMotorAmps();

    public abstract void setSetpoint(double meters);

    public abstract double getPosition();

    public abstract double getVelocity();

    public abstract double getAcceleration();

    public Command goToSetSetpoint(Supplier<Double> position){
        return run(
            () -> {

            }
        );
    }
}






//miles was here
