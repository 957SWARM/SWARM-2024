package com.team957.comp2024.input;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class DefaultDriver implements DriverInput {

    // implementation detail of the driver profile
    // example: slower profile for outreach events
    private static final double LIN_MAX_SPEED = 6;
    private static final double ROT_MAX_SPEED = 10;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(17);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(17);
    private final SlewRateLimiter angularLimiter = new SlewRateLimiter(30);

    private final XboxController xboxController;

    public DefaultDriver() {

        xboxController = new XboxController(0);
    }

    @Override
    public double swerveX() {
        // consistent polling rate so this is fine??
        return xLimiter.calculate(LIN_MAX_SPEED * xboxController.getLeftY());
    }

    @Override
    public double swerveY() {
        return yLimiter.calculate(LIN_MAX_SPEED * xboxController.getLeftX());
    }

    @Override
    public boolean zeroGyro() {
        return xboxController.getBackButton();
    }

    @Override
    public double swerveRot() {
        return angularLimiter.calculate(ROT_MAX_SPEED * xboxController.getRightX());
    }

    @Override
    public boolean enableTracking() {
        return xboxController.getRightBumper();
    }
}
