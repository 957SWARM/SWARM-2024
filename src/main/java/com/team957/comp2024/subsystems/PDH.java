package com.team957.comp2024.subsystems;

import com.team957.comp2024.Constants;
import com.team957.comp2024.Robot;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import monologue.Annotations.Log;
import monologue.Logged;

public class PDH implements Subsystem, Logged {
    // only create log identifier strings once
    // since Java string concatenation is absurdly slow and we actually care about loop times
    private final HashMap<Integer, String> channelNumToCurrentLogName = new HashMap<>();

    private final PowerDistribution pdh =
            new PowerDistribution(Constants.PDHConstants.PDH_CAN_ID, ModuleType.kRev);

    private boolean switchableChannelState;

    public PDH(boolean switchableChannelState) {
        for (int i = 0; i < pdh.getNumChannels(); i++) {
            String dirName = "channel" + String.valueOf(i);

            channelNumToCurrentLogName.put(i, dirName + "/currentAmps");
        }

        this.switchableChannelState = switchableChannelState;
        setSwitchableChannel(switchableChannelState);

        register();
    }

    public double getCurrentAmps(int channel) {
        return pdh.getCurrent(channel);
    }

    @Log.NT
    public double getTemperatureC() {
        return pdh.getTemperature();
    }

    @Log.NT
    public double getTotalCurrentAmps() {
        return pdh.getTotalCurrent();
    }

    @Log.NT
    public double getTotalEnergyJoules() {
        return pdh.getTotalEnergy();
    }

    @Log.NT
    public double getVoltage() {
        return pdh.getVoltage();
    }

    public void setSwitchableChannel(boolean on) {
        pdh.setSwitchableChannel(on);
        switchableChannelState = on;
    }

    /**
     * This is based on the RIO-side stored state, and is not necessarily the actual state of the
     * PDH, especially near startups / resets.
     *
     * @return The state of the switchable channel, `true` if current can flow.
     */
    @Log.NT
    public boolean getSwitchableChannelState() {
        return switchableChannelState;
    }

    @Override
    public void periodic() {
        Robot.ui.setTotalCurrentDraw(getTotalCurrentAmps());

        for (int i = 0; i < pdh.getNumChannels(); i++) {
            log(channelNumToCurrentLogName.get(i), getCurrentAmps(i));
            // TODO: support for faults once next WPILib released
            // which can get faults by index
        }
    }
}
