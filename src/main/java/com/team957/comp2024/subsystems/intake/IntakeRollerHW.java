package com.team957.comp2024.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team957.comp2024.Constants.IntakeRollerConstants;

public class IntakeRollerHW extends IntakeRoller {

    private final CANSparkMax roller = new CANSparkMax(IntakeRollerConstants.ROLLER_CANID, MotorType.kBrushless);
    
    public IntakeRollerHW(){
        roller.restoreFactoryDefaults();

        roller.setSmartCurrentLimit(IntakeRollerConstants.CURRENT_LIMIT);

        // intaking = positive, puking = negative
        roller.setInverted(IntakeRollerConstants.ROLLER_INVERTED);
    }
    @Override
    public void setRollerVoltage(double voltage) {
        roller.set(voltage);
    }
    @Override
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }
    @Override
    public double getRollerAmps() {
        return roller.getOutputCurrent();
    }

    
}
