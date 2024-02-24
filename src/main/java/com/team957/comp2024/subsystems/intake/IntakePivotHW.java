package com.team957.comp2024.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.team957.comp2024.Constants;
import com.team957.comp2024.util.SparkMaxUtils;
import com.team957.lib.math.UtilityMath;
import monologue.Annotations.Log;

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

    private final SparkPIDController controller = motor.getPIDController();

    public IntakePivotHW() {
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(Constants.IntakePivotConstants.INTAKE_PIVOT_CURRENT_LIMIT_AMPS);

        motor.setInverted(Constants.IntakePivotConstants.INTAKE_PIVOT_MOTOR_INVERTED);

        controller.setFeedbackDevice(encoder);

        controller.setPositionPIDWrappingEnabled(true);
        controller.setPositionPIDWrappingMinInput(0);
        controller.setPositionPIDWrappingMaxInput(1);

        controller.setP(Constants.IntakePivotConstants.ONBOARD_CONTROLLER_KP);
        controller.setI(Constants.IntakePivotConstants.ONBOARD_CONTROLLER_KI);
        controller.setD(Constants.IntakePivotConstants.ONBOARD_CONTROLLER_KD);
    }

    @Override
    protected void setVoltageUnsafe(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public double getPositionRadians() {
        return UtilityMath.normalizeAngleRadians(
                getUnoffsetPositionRadians()
                        - Constants.IntakePivotConstants.INTAKE_PIVOT_OFFSET_RADIANS);
    }

    @Log.NT
    public double getUnoffsetPositionRadians() {
        return encoder.getPosition() * Math.PI * 2;
    }

    @Override
    public double getVelocityRadiansPerSecond() {
        return encoder.getVelocity() * Math.PI * 2;
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
    protected void setFeedforwardAndSetpointUnsafe(double volts, double setpointRadians) {
        double unoffset =
                setpointRadians + Constants.IntakePivotConstants.INTAKE_PIVOT_OFFSET_RADIANS;

        controller.setReference(unoffset / (2 * Math.PI), ControlType.kPosition, 0, volts);
        // System.out.println("unoffset: " + unoffset);
        // System.out.println("offset: " + setpointRadians);
    }

    @Override
    public void periodic() {
        super.periodic();

        // not required to override this, but want to prevent accidentially overriding the code in
        // superclass
    }
}
