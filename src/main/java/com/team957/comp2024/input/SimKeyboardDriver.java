package com.team957.comp2024.input;

import edu.wpi.first.wpilibj.XboxController;

public class SimKeyboardDriver implements DriverInput {
    private final XboxController controller = new XboxController(0);

    private final double maxLinearSpeed = 6; // m/s
    private final double maxRotationalSpeed = 10; // rad/s

    @Override
    public double swerveX() {
        return maxLinearSpeed * -controller.getLeftY();
    }

    @Override
    public double swerveY() {
        return maxLinearSpeed * -controller.getLeftX();
    }

    @Override
    public boolean zeroGyro() {
        return false;
    }

    @Override
    public double swerveRot() {
        return maxRotationalSpeed * controller.getLeftTriggerAxis();
    }

    @Override
    public boolean noteTracking() {
        return controller.getRightBumper();
    }

    @Override
    public boolean speaker() {
        return false;
    }

    @Override
    public boolean intake() {
        return false;
    }

    @Override
    public boolean raiseHook() {
        return false;
    }

    @Override
    public boolean lowerHook() {
        return false;
    }

    @Override
    public boolean climb() {
        return false;
    }

    @Override
    public boolean floorIntake() {
        return false;
    }

    @Override
    public boolean enableAprilTagTracking() {
        return false;
    }

    @Override
    public boolean amp() {
        return false;
    }
}
