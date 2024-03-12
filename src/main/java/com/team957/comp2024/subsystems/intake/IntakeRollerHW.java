package com.team957.comp2024.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team957.comp2024.Constants;
import com.team957.comp2024.Constants.IntakeRollerConstants;
import com.team957.comp2024.util.FaultDetectionUtil.StaleNoisyDataDetector;
import com.team957.comp2024.util.SparkMaxUtils;
import com.team957.comp2024.util.SparkMaxUtils.SparkMaxAlertsUtil;

public class IntakeRollerHW extends IntakeRoller {
    private double tofReadingMeters = 0;

    private final StaleNoisyDataDetector detector =
            new StaleNoisyDataDetector(
                    "Time of flight range", IntakeRollerConstants.STALENESS_THRESHOLD_CYCLES);

    private final CANSparkMax roller =
            SparkMaxUtils.slowUnusedPeriodics(
                    new CANSparkMax(
                            Constants.IntakeRollerConstants.ROLLER_CANID, MotorType.kBrushless),
                    false,
                    true,
                    true,
                    true,
                    true);

    private final TimeOfFlight tof = new TimeOfFlight(IntakeRollerConstants.TOF_CANID);

    private final SparkMaxAlertsUtil util =
            new SparkMaxAlertsUtil(roller, "intake roller", IntakeRollerConstants.CURRENT_LIMIT);

    public IntakeRollerHW() {
        tof.setRangingMode(RangingMode.Short, IntakeRollerConstants.TOF_TIMING_BUDGET_MS);

        roller.restoreFactoryDefaults();

        roller.setSmartCurrentLimit(IntakeRollerConstants.CURRENT_LIMIT);

        // intaking = positive, puking = negative
        roller.setInverted(IntakeRollerConstants.ROLLER_INVERTED);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        roller.setVoltage(voltage);
    }

    @Override
    public double getRollerAmps() {
        return roller.getOutputCurrent();
    }

    @Override
    public double getAppliedVoltage() {
        return roller.getAppliedOutput() * roller.getBusVoltage();
    }

    @Override
    public boolean noteIsPresent() {
        return (tofReadingMeters <= IntakeRollerConstants.TOF_NOTE_PRESENT_THRESHOLD_METERS);
    }

    @Override
    public void setSimulationNoteIsPresentMock(boolean mock) {}

    // may be stale
    @Override
    public double getTofReadingMeters() {
        return tofReadingMeters;
    }

    @Override
    public void periodic() {
        super.periodic();

        tofReadingMeters = tof.getRange() / 1000;

        detector.poll(tofReadingMeters);

        util.poll();
    }
}
