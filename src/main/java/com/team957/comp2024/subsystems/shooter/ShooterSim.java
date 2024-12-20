package com.team957.comp2024.subsystems.shooter;

import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.Constants.ShooterConstants;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final FlywheelSim sim =
            new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(1, 2),
                    ShooterConstants.SHOOTER_MOTOR,
                    ShooterConstants.SHOOTER_REDUCTION);

    private double volts = 0;

    @Override
    public void setShooterVoltage(double voltage) {
        sim.setInputVoltage(voltage);
        volts = voltage;
    }

    @Override
    public double getLeftMotorVoltage() {
        return volts;
    }

    @Override
    public double getRightMotorVoltage() {
        return volts;
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

    @Override
    public void periodic() {
        super.periodic();

        sim.update(MiscConstants.NOMINAL_LOOP_TIME_SECONDS);
    }
}
