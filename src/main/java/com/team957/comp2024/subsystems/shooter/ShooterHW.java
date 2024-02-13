package com.team957.comp2024.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team957.comp2024.Constants.ShooterConstants;

public class ShooterHW extends Shooter {

    private final CANSparkMax leftMotor =
            new CANSparkMax(ShooterConstants.LEFT_CANID, MotorType.kBrushless);
    private final CANSparkMax rightMotor =
            new CANSparkMax(ShooterConstants.RIGHT_CANID, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    public ShooterHW() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        // set inversions
        leftMotor.setInverted(ShooterConstants.leftMotorInverted);
        rightMotor.setInverted(ShooterConstants.rightMotorInverted);

        leftMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

        // sets the leftMotor as the master and the rightMotor as follower
        rightMotor.follow(leftMotor);
    }

    @Override
    public void setShooterVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public double getLeftMotorVoltage() {
        return leftMotor.getBusVoltage();
    }

    @Override
    public double getRightMotorVoltage() {
        return rightMotor.getBusVoltage();
    }

    @Override
    public double getLeftMotorAmps() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public double getRightMotorAmps() {
        return rightMotor.getOutputCurrent();
    }

    @Override
    public double getVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        // not required to override this, but want to prevent accidentially overriding the code in
        // superclass
    }
}
