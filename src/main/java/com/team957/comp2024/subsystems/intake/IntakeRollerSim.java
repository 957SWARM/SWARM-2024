package com.team957.comp2024.subsystems.intake;

import java.util.function.Supplier;

public class IntakeRollerSim extends IntakeRoller {
    private Supplier<Boolean> noteIsPresentMock = null;

    private double setVolts = 0;

    @Override
    public void setRollerVoltage(double voltage) {
        setVolts = voltage;
    }

    @Override
    public double getAppliedVoltage() {
        return setVolts;
    }

    @Override
    public boolean noteIsPresent() {
        return (noteIsPresentMock == null) ? false : noteIsPresentMock.get();
    }

    @Override
    public double getRollerAmps() {
        return 0;
    }

    @Override
    public void setSimulationNoteIsPresentMock(Supplier<Boolean> mock) {
        this.noteIsPresentMock = mock;
    }
}
