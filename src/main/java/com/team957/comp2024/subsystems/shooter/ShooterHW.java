package com.team957.comp2024.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team957.comp2024.Constants.ShooterConstants;
import com.team957.comp2024.util.SparkMaxUtils;
import com.team957.comp2024.util.SparkMaxUtils.SparkMaxAlertsUtil;

public class ShooterHW extends Shooter {

    private final CANSparkMax leftMotor =
            SparkMaxUtils.slowUnusedPeriodics(
                    new CANSparkMax(ShooterConstants.LEFT_CANID, MotorType.kBrushless),
                    true,
                    true,
                    true,
                    true,
                    true);

    private final CANSparkMax rightMotor =
            SparkMaxUtils.slowUnusedPeriodics(
                    new CANSparkMax(ShooterConstants.RIGHT_CANID, MotorType.kBrushless),
                    true,
                    true,
                    true,
                    true,
                    true);

    private final SparkMaxAlertsUtil rightUtil =
            new SparkMaxAlertsUtil(rightMotor, "shooter right", ShooterConstants.CURRENT_LIMIT);
    private final SparkMaxAlertsUtil leftUtil =
            new SparkMaxAlertsUtil(leftMotor, "shooter left", ShooterConstants.CURRENT_LIMIT);

    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    public ShooterHW() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
    }

    @Override
    public void setLeftVoltage(double voltage) {
        leftMotor.setVoltage(ShooterConstants.leftMotorInverted ? -voltage : voltage);
    }

    @Override
    public void setRightVoltage(double voltage) {
        rightMotor.setVoltage(ShooterConstants.rightMotorInverted ? -voltage : voltage);
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
    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        super.periodic();

        leftUtil.poll();
        rightUtil.poll();
    }
}
