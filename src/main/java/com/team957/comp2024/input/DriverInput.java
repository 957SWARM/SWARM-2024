package com.team957.comp2024.input;

public interface DriverInput {

    // in meters/sec
    public double swerveX();

    // in meters/sec
    public double swerveY();

    // on rising edge
    public boolean zeroGyro();

    // in radians/sec
    public double swerveRot();

    // in true or false
    public boolean noteTracking();

    // shoot
    public boolean speakerSequence();

    // winch upwards!
    public boolean climbWinch();

    // slowly intakes
    public boolean slowIntake();

    // slowly ejects
    public boolean slowEject();

    // brings intake to floor
    public boolean intakeSequence();

    public void setRumble(boolean on);

    public boolean intakePivotStow();

    public boolean shootAmp();

    public boolean pivotAmp();

    public boolean activeNoteCentering();
}
