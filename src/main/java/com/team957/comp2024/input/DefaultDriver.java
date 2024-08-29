package com.team957.comp2024.input;

import com.team957.comp2024.Constants.OIConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class DefaultDriver implements DriverInput {

    // implementation detail of the driver profile
    // example: slower profile for outreach events
    private static final double LIN_MAX_SPEED = 1.6;
    private static final double ROT_MAX_SPEED = 4;

    private final XboxController xboxController;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(20);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(20);
    private final SlewRateLimiter angularLimiter = new SlewRateLimiter(2000);

    public DefaultDriver() {
        xboxController = new XboxController(OIConstants.DRIVER_PORT);
    }

    @Override
    public double swerveX() {
        // consistent polling rate so this is fine??
        double output =
                Math.signum(xboxController.getLeftY()) * Math.pow(xboxController.getLeftY(), 2);

        double invert = (DriverStation.getAlliance().get() == Alliance.Red) ? -1 : 1;

        return xLimiter.calculate(LIN_MAX_SPEED * invert * output);
    }

    @Override
    public double swerveY() {
        double output =
                Math.signum(xboxController.getLeftX()) * Math.pow(xboxController.getLeftX(), 2);

        double invert = (DriverStation.getAlliance().get() == Alliance.Red) ? -1 : 1;

        return yLimiter.calculate(LIN_MAX_SPEED * invert * output);
    }

    @Override
    public boolean zeroGyro() {
        return xboxController.getRawButton(8);
    }

    @Override
    public double swerveRot() {
        double invert = (DriverStation.getAlliance().get() == Alliance.Red) ? -1 : 1;

        return angularLimiter.calculate(ROT_MAX_SPEED * xboxController.getRightX());
    }

    @Override
    public boolean speakerSequence() {
        return xboxController.getYButton();
    }

    @Override
    public boolean intakeSequence() {
        return xboxController.getAButton();
    }

    @Override
    public boolean intakePivotStow() {
        return xboxController.getXButton();
    }

    @Override
    public boolean slowEject() {
        return xboxController.getRightBumper();
    }

    @Override
    public boolean slowIntake() {
        return xboxController.getLeftBumper();
    }

    @Override
    public boolean climbWinch() {
        return xboxController.getRawButton(7);
    }

    @Override
    public boolean noteTracking() {
        return (xboxController.getLeftTriggerAxis() > .5);
    }

    @Override
    public boolean shootAmp() {
        return (xboxController.getRightTriggerAxis() > .5);
    }

    @Override
    public void setRumble(boolean on) {
        xboxController.setRumble(RumbleType.kBothRumble, on ? 1 : 0);
    }

    @Override
    public boolean pivotAmp() {
        return xboxController.getBButton();
    }

    @Override
    public boolean activeNoteCentering() {
        return (xboxController.getPOV() == 270);
    }
}
