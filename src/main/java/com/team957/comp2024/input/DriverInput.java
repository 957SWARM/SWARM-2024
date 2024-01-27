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

    // shoot
    public boolean shoot();
}
