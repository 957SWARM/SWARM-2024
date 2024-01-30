package com.team957.comp2024.subsystems.boxClimber;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public abstract class BoxClimber implements Subsystem, Logged{
    public abstract void setMotorVoltage(double voltage);

    public abstract double getMotorVoltage();

    public abstract double getMotorAmps();

    // puts the climber in rest
    public Command idleCommand(){
        return run(
            () -> {
                setMotorVoltage(0);
            }
        );
    }

    // raises the climber
    public Command raiseCommand(){
        return run(
            () -> {
                setMotorVoltage(ClimberConstants.STANDARD_VOLTAGE);
            }
        );
    }

    // lowers the climber
    public Command lowerCommand(){
        return run(
            () -> {
                setMotorVoltage(-ClimberConstants.STANDARD_VOLTAGE);
            }
        );
    }
}
//miles was here
