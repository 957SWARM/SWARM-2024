package com.team957.comp2024.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.team957.comp2024.Constants;
import com.team957.lib.math.UtilityMath;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Annotations.Log;
import monologue.Logged;

public class Pivot implements Subsystem, Logged {
    CANSparkMax pivot =
            new CANSparkMax(
                    Constants.IntakePivotConstants.INTAKE_PIVOT_MOTOR_CANID, MotorType.kBrushless);
    AbsoluteEncoder encoder = pivot.getAbsoluteEncoder();
    SparkPIDController pid;

    public Pivot() {
        register();
        pivot.restoreFactoryDefaults();
        pivot.setInverted(false);
        pid = pivot.getPIDController();
        pid.setFeedbackDevice(encoder);
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMaxInput(Constants.PivotConstants.PID_WRAP_MAX);
        pid.setPositionPIDWrappingMinInput(Constants.PivotConstants.PID_WRAP_MIN);
        pid.setP(Constants.PivotConstants.ONBOARD_KP);
        pid.setI(Constants.PivotConstants.ONBOARD_KI);
        pid.setD(Constants.PivotConstants.ONBOARD_KD);
    }

    public void setSetpoint(double setPoint, double FF) {

        // Get current encoder position

        // get difference between 2

        // check if diff is > abs .4

        //

        pid.setReference(setPoint, ControlType.kPosition, 0, FF);
    }

    @Log.NT
    public double getOffsetRev() {
        double radians =
                (encoder.getPosition()
                                - Constants.PivotConstants.OFFSET_REV
                                + Constants.PivotConstants.OFFSET_TO_STRAIGHT)
                        * 2
                        * Math.PI;
        return UtilityMath.normalizeAngleRadians(radians) / (2 * Math.PI);
    }

    @Log.NT
    public double getUnoffsetRev() {
        return encoder.getPosition();
    }

    @Log.NT
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
