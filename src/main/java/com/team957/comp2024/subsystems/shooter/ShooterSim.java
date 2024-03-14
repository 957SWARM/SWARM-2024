package com.team957.comp2024.subsystems.shooter;

import com.team957.comp2024.Constants.MiscConstants;
import com.team957.comp2024.Constants.ShooterConstants;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final FlywheelSim leftSim =
            new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(1, 2),
                    ShooterConstants.SHOOTER_MOTOR,
                    ShooterConstants.SHOOTER_REDUCTION);

    private final FlywheelSim rightSim =
            new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(1, 2),
                    ShooterConstants.SHOOTER_MOTOR,
                    ShooterConstants.SHOOTER_REDUCTION);

    private double leftVolts = 0;
    private double rightVolts = 0;

    @Override
    public void setLeftVoltage(double voltage) {
        leftSim.setInputVoltage(voltage);
        leftVolts = voltage;
    }

    @Override
    public void setRightVoltage(double voltage) {
        rightSim.setInputVoltage(voltage);
        rightVolts = voltage;
    }

    @Override
    public double getLeftMotorVoltage() {
        return leftVolts;
    }

    @Override
    public double getRightMotorVoltage() {
        return rightVolts;
    }

    @Override
    public double getLeftMotorAmps() {
        return leftSim.getCurrentDrawAmps();
    }

    @Override
    public double getRightMotorAmps() {
        return rightSim.getCurrentDrawAmps();
    }

    @Override
    public double getLeftVelocity() {
        return leftSim.getAngularVelocityRPM();
    }

    @Override
    public double getRightVelocity() {
        return rightSim.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        super.periodic();

        leftSim.update(MiscConstants.NOMINAL_LOOP_TIME_SECONDS);
        rightSim.update(MiscConstants.NOMINAL_LOOP_TIME_SECONDS);
    }
}
