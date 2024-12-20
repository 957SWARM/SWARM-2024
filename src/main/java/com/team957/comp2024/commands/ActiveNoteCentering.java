package com.team957.comp2024.commands;

import com.team957.comp2024.Constants.IntakeRollerConstants;
import com.team957.comp2024.subsystems.intake.IntakeRoller;
import edu.wpi.first.wpilibj2.command.Command;

public class ActiveNoteCentering extends Command {
    // gets its own class because it has soo much state

    private final IntakeRoller roller;

    private boolean hasReversed = false;

    private double lastTofMeasurement = 957;

    private boolean centered = false;

    public ActiveNoteCentering(IntakeRoller roller) {
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public void execute() {
        if (roller.getTofReadingMeters() == lastTofMeasurement) {
            // interpret no change in range as stale reading
            // stale readings very possible because of fast loop time and slow sensor updates
            roller.setRollerVoltage(0);
        } else if (roller.getTofReadingMeters()
                > lastTofMeasurement + IntakeRollerConstants.CENTERING_REVERSAL_HYSTERESIS_METERS) {
            // if has increased more than hysteresis amount since last real reading
            lastTofMeasurement = roller.getTofReadingMeters();

            // if we're going up, and have already flipped, stop to get close to center
            if (hasReversed) {
                centered = true;
            } else {
                // otherwise flip
                hasReversed = true;

                roller.setRollerVoltage(-IntakeRollerConstants.CENTERING_PULSE_VOLTAGE);
            }
        } else {
            // if this is reached, there is either not enough movement, or moving in the right
            // direction already
            lastTofMeasurement = roller.getTofReadingMeters();

            roller.setRollerVoltage(
                    IntakeRollerConstants.CENTERING_PULSE_VOLTAGE * (hasReversed ? -1 : 1));
        }
    }

    @Override
    public boolean isFinished() {
        return centered;
    }
}
