package com.team957.comp2024.subsystems.boxClimber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team957.comp2024.Constants.ClimberConstants;

public class BoxClimberHW extends BoxClimber{
    
    private final CANSparkMax climbMotor = new CANSparkMax(ClimberConstants.MOTOR_CANID, MotorType.kBrushless);
    //private final RelativeEncoder encoder; (maybe add later?)
    // possibly solenoids in the future?
    public BoxClimberHW(){
    }

    @Override
    public void setMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    @Override
    public double getMotorVoltage() {
        return climbMotor.getBusVoltage();
    }

    @Override
    public double getMotorAmps() {
        return climbMotor.getOutputCurrent();
    }

}