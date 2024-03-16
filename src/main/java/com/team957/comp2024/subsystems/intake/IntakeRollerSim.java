package com.team957.comp2024.subsystems.intake;

public class IntakeRollerSim extends IntakeRoller {
    private boolean noteIsPresentMock = true; // start match with note

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
        return noteIsPresentMock;
    }

    @Override
    public double getTofReadingMeters() {
        return 0;
    }

    @Override
    public double getRollerAmps() {
        return 0;
    }

    @Override
    public void setSimulationNoteIsPresentMock(boolean mock) {
        this.noteIsPresentMock = mock;
    }

    @Override
    public void periodic() {
        super.periodic();

        // not required to override this, but want to prevent accidentially overriding the code in
        // superclass
    }
}
