package com.team957.comp2024.subsystems;

import com.team957.comp2024.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public class PneumaticsHub implements Subsystem, Logged {
    public PneumaticsHub() {}

    private final PneumaticHub ph = new PneumaticHub(Constants.PneumaticHubConstants.PH_CAN_ID);

    public double getCompressorCurrentAmps() {
        return ph.getCompressorCurrent();
    }

    public double getAnalogPressurePSI() {
        return ph.getPressure(Constants.PneumaticHubConstants.PRESSURE_SENSOR_ANALOG_CHANNEL);
    }

    public double getTotalSolenoidCurrentAmps() {
        return ph.getSolenoidsTotalCurrent();
    }

    public boolean getDigitalPressureSwitch() {
        return ph.getPressureSwitch();
    }

    public double getInputVoltage() {
        return ph.getInputVoltage();
    }

    public DoubleSolenoid getDoubleSolenoid(int forwardChannel, int backwardsChannel) {
        return ph.makeDoubleSolenoid(forwardChannel, backwardsChannel);
    }

    public Solenoid getSolenoid(int channel) {
        return ph.makeSolenoid(channel);
    }
}
