package com.team957.comp2024.subsystems;

import com.team957.comp2024.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public class PDH implements Subsystem, Logged {
    public PDH() {}

    private final PowerDistribution pdh =
            new PowerDistribution(Constants.PDHConstants.PDH_CAN_ID, ModuleType.kRev);

    public double getCurrentAmps(int channel) {
        return pdh.getCurrent(channel);
    }

    public double getTemperatureC() {
        return pdh.getTemperature();
    }

    public double getTotalCurrentAmps() {
        return pdh.getTotalCurrent();
    }

    public double getTotalEnergyJoules() {
        return pdh.getTotalEnergy();
    }

    public double getVoltage() {
        return pdh.getVoltage();
    }

    public void setSwitchableChannel(boolean on) {
        pdh.setSwitchableChannel(on);
    }
}
