package com.team957.comp2024.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team957.comp2024.Constants.ShooterConstants;
import com.team957.comp2024.util.SparkMaxUtils;

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

    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    public ShooterHW() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
    }

    @Override
    public void setShooterVoltage(double voltage) {
        leftMotor.setVoltage(ShooterConstants.leftMotorInverted ? -voltage : voltage);
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
    public double getVelocity() {
        return 2 * Math.PI * (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        // not required to override this, but want to prevent accidentially overriding the code in
        // superclass
    }
}
