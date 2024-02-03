package com.team957.comp2024.subsystems.climbing;

import com.team957.comp2024.Constants.WinchConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public abstract class Winch implements Subsystem, Logged{
    public abstract void setWinchVoltage(double voltage);

    public abstract double getWinchVoltage();

    public abstract double getWinchAmps();

    public static Winch getWinch(boolean isReal){
        return (isReal) ? new WinchHW() : WinchSim();
    }

    // puts the climber in rest
    public Command idleCommand(){
        return run(
            () -> {
                setWinchVoltage(0);
            }
        );
    }

    // raises the winch
    public Command raiseCommand(){
        return run(
            () -> {
                setWinchVoltage(WinchConstants.STANDARD_VOLTAGE);
            }
        );
    }

}
