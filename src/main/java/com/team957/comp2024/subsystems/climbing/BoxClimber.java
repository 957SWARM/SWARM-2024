package com.team957.comp2024.subsystems.climbing;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.BoxClimberConstants;

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
                setMotorVoltage(BoxClimberConstants.STANDARD_VOLTAGE);
            }
        );
    }

    // lowers the climber
    public Command lowerCommand(){
        return run(
            () -> {
                setMotorVoltage(-BoxClimberConstants.STANDARD_VOLTAGE);
            }
        );
    }

    public static BoxClimber getBoxClimber(boolean isReal){
        return (isReal) ? new BoxClimberHW() : new BoxClimberSim();
    }
}
//miles was here
