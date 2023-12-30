package com.team957.comp2024.input;

import edu.wpi.first.wpilibj.XboxController;

public class DefaultDriver implements DriverInput {

    // implementation detail of the driver profile
    // example: slower profile for outreach events
    private static final double LIN_MAX_SPEED = 6;
    private static final double ROT_MAX_SPEED = 10;

    private final XboxController xboxController;

    public DefaultDriver(int port) {

        xboxController = new XboxController(0);
    }

    @Override
    public double swerveX() {
        return LIN_MAX_SPEED * xboxController.getLeftY();
    }

    @Override
    public double swerveY() {
        return LIN_MAX_SPEED * -xboxController.getLeftX();
    }

    @Override
    public boolean zeroGyro() {
        return xboxController.getBackButton();
    }

    @Override
    public double swerveRot() {
        return -ROT_MAX_SPEED * xboxController.getRightX();
    }
}
