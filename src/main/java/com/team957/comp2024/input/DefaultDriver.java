package com.team957.comp2024.input;

import com.team957.comp2024.Constants.OIConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class DefaultDriver implements DriverInput {

    // implementation detail of the driver profile
    // example: slower profile for outreach events
    private static final double LIN_MAX_SPEED = 6;
    private static final double ROT_MAX_SPEED = 10;

    private final XboxController xboxController;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(20);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(20);
    private final SlewRateLimiter angularLimiter = new SlewRateLimiter(20);

    public DefaultDriver() {
        xboxController = new XboxController(OIConstants.DRIVER_PORT);
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
        return xboxController.getRawButton(8);
    }

    @Override
    public double swerveRot() {
        return angularLimiter.calculate(ROT_MAX_SPEED * xboxController.getRightX());
    }

    @Override
    public boolean speaker() {
        return xboxController.getYButton();
    }

    @Override
    public boolean intake() {
        return xboxController.getXButton();
    }

    @Override
    public boolean raiseHook() {
        return xboxController.getRightBumper();
    }

    @Override
    public boolean lowerHook() {
        return xboxController.getLeftBumper();
    }

    @Override
    public boolean climb() {
        return xboxController.getBButton();
    }

    @Override
    public boolean floorIntake() {
        return xboxController.getXButton();
    }

    @Override
    public boolean noteTracking() {
        return (xboxController.getRightTriggerAxis() > .5);
    }

    @Override
    public boolean enableAprilTagTracking() {
        return (xboxController.getLeftTriggerAxis() > .5);
    }

    @Override
    public boolean amp() {
        return (xboxController.getPOV() == 0);
    }
}
