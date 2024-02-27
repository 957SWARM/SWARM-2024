package com.team957.comp2024.subsystems.climbing;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.team957.comp2024.Constants.WinchConstants;

public class WinchHW extends Winch {
    private final TalonSRX winch =
            new TalonSRX(WinchConstants.MOTOR_CANID);

    public WinchHW() {
        winch.configFactoryDefault();

        winch.enableCurrentLimit(true);
        winch.configPeakCurrentLimit(WinchConstants.CURRENT_LIMIT);

        winch.setInverted(WinchConstants.MOTOR_INVERTED);
    }

    @Override
    public void setWinchVoltage(double voltage) {
        winch.set(TalonSRXControlMode.PercentOutput, voltage / 12);
    }

    @Override
    public double getWinchVoltage() {
        return winch.getBusVoltage();
    }

    @Override
    public double getWinchAmps() {
        return winch.getSupplyCurrent();
    }

    @Override
    public void periodic() {
        super.periodic();

        // not required to override this, but want to prevent accidentially overriding the code in
        // superclass
    }
}
