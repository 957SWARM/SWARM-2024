package com.team957.comp2024.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.IntakePivotConstants;
import com.team957.comp2024.util.SparkMaxUtils;
import com.team957.comp2024.util.SparkMaxUtils.SparkMaxAlertsUtil;
import com.team957.lib.math.UtilityMath;

public class IntakePivotHW extends IntakePivot {
    private final CANSparkMax motor =
            SparkMaxUtils.slowUnusedPeriodics(
                    new CANSparkMax(
                            Constants.IntakePivotConstants.INTAKE_PIVOT_MOTOR_CANID,
                            MotorType.kBrushless),
                    true,
                    true,
                    true,
                    false,
                    false);

    private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

    private final SparkMaxAlertsUtil util =
            new SparkMaxAlertsUtil(
                    motor, "intake pivot", IntakePivotConstants.INTAKE_PIVOT_CURRENT_LIMIT_AMPS);

    public IntakePivotHW() {
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(Constants.IntakePivotConstants.INTAKE_PIVOT_CURRENT_LIMIT_AMPS);

        motor.setInverted(Constants.IntakePivotConstants.INTAKE_PIVOT_MOTOR_INVERTED);

        encoder.setPositionConversionFactor(2 * Math.PI);

        motor.burnFlash();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public double getPositionRadians() {
        return UtilityMath.normalizeAngleRadians(
                encoder.getPosition() + Constants.IntakePivotConstants.INTAKE_PIVOT_OFFSET_RADIANS);
    }

    @Override
    public double getVelocityRadiansPerSecond() {
        return encoder.getVelocity();
    }

    @Override
    public double getCurrentAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getControlEffortVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void periodic() {
        super.periodic();

        util.poll();
    }
}
