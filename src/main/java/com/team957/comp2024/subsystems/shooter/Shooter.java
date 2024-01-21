package com.team957.comp2024.subsystems.shooter;

import com.team957.comp2024.subsystems.swerve.Swerve;
import com.team957.comp2024.subsystems.swerve.SwerveHW;
import com.team957.comp2024.subsystems.swerve.SwerveSim;

import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Shooter implements Subsystem, Logged {
    public abstract static class ShooterIO implements Logged {
        protected ShooterIO() {}

        // sets the applied voltage to both motors
        public abstract void setMotorVoltage(double voltage);

        // returns the current voltage of the motor
        @Log.NT
        public abstract double getMotorVoltage();

    }
    // will set the RPM shooter attempts to reach
    public abstract void setVelocitySetpoint(double RPM);

    // returns the target RPM of the shooter
    @Log.NT
    public abstract double getVelocitySetpoint();

    // returns the current RPM of the shooter
    @Log.NT
    public abstract double getVelocityCurrent();

    // returns the current amps of the motor
    @Log.NT
    public abstract double getMotorAmps();


    protected Shooter(ShooterIO leftMotor, ShooterIO rightMotor){

    }

    /*
    public static Shooter getShooter(boolean isReal) {
        if(isReal)
            return new ShooterHW();
        else
            return new ShooterSim();
    }
    */
}
