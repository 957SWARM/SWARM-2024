package com.team957.comp2024.subsystems;

import com.team957.comp2024.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Annotations.Log;
import monologue.Logged;

public class PneumaticsHub implements Subsystem, Logged {
    public PneumaticsHub() {
        for (int i = 0; ph.checkSolenoidChannel(i); i++) {
            String dirName = "channel" + String.valueOf(i);
            // unused; waiting for wpilib support of getting faults by index
        }
    }

    private final PneumaticHub ph = new PneumaticHub(Constants.PneumaticHubConstants.PH_CAN_ID);

    @Log.NT
    public double getCompressorCurrentAmps() {
        return ph.getCompressorCurrent();
    }

    @Log.NT
    public double getAnalogPressurePSI() {
        return ph.getPressure(Constants.PneumaticHubConstants.PRESSURE_SENSOR_ANALOG_CHANNEL);
    }

    @Log.NT
    public double getTotalSolenoidCurrentAmps() {
        return ph.getSolenoidsTotalCurrent();
    }

    @Log.NT
    public boolean getDigitalPressureSwitch() {
        return ph.getPressureSwitch();
    }

    @Log.NT
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
