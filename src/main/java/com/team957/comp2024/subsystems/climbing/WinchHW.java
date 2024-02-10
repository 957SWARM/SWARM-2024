package com.team957.comp2024.subsystems.climbing;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team957.comp2024.Constants.WinchConstants;

public class WinchHW extends Winch {
    private final CANSparkMax winch =
            new CANSparkMax(WinchConstants.MOTOR_CANID, MotorType.kBrushless);

    public WinchHW() {
        winch.restoreFactoryDefaults();

        winch.setSmartCurrentLimit(WinchConstants.CURRENT_LIMIT);

        winch.setInverted(WinchConstants.MOTOR_INVERTED);
    }

    @Override
    public void setWinchVoltage(double voltage) {
        winch.setVoltage(voltage);
    }

    @Override
    public double getWinchVoltage() {
        return winch.getBusVoltage();
    }

    @Override
    public double getWinchAmps() {
        return winch.getOutputCurrent();
    }
}
