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

    public boolean enableAprilTagTracking();

    // shoot
    public boolean speaker();

    // pivots intake
    public boolean intake();

    // raise hook
    public boolean raiseHook();

    // lower hooK?
    public boolean lowerHook();

    // winch upwards!
    public boolean climb();

    // take the note in
    public boolean floorIntake();

    // brings pivot to amp position
    public boolean amp();
}
