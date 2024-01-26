package com.team957.comp2024.subsystems.shooter;

import com.team957.comp2024.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final FlywheelSim sim = new FlywheelSim(null, ShooterConstants.SHOOTER_MOTOR, 0);

    @Override
    public void setShooterVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getLeftMotorVoltage() {
        return 0;
    }

    @Override
    public double getRightMotorVoltage() {
        return 0;
    }

    @Override
    public double getLeftMotorAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public double getRightMotorAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public double getVelocity() {
        return sim.getAngularVelocityRPM();
    }


}