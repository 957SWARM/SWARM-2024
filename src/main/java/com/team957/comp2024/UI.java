package com.team957.comp2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class UI implements Logged {
    private SwerveModuleState[] moduleStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };

    private double rotationDegrees = 0;

    private double currentAmps = 0;
    private double voltage = 0;

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final Sendable swerveVis =
            new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                    builder.setSmartDashboardType("SwerveDrive");

                    builder.addDoubleProperty(
                            "Front Left Angle", () -> moduleStates[0].angle.getDegrees(), null);
                    builder.addDoubleProperty(
                            "Front Left Velocity",
                            () -> moduleStates[0].speedMetersPerSecond,
                            null);

                    builder.addDoubleProperty(
                            "Front Right Angle", () -> moduleStates[1].angle.getDegrees(), null);

                    builder.addDoubleProperty(
                            "Front Right Velocity",
                            () -> moduleStates[1].speedMetersPerSecond,
                            null);

                    builder.addDoubleProperty(
                            "Back Right Angle", () -> moduleStates[2].angle.getDegrees(), null);
                    builder.addDoubleProperty(
                            "Back Right Velocity",
                            () -> moduleStates[2].speedMetersPerSecond,
                            null);

                    builder.addDoubleProperty(
                            "Back Left Angle", () -> moduleStates[3].angle.getDegrees(), null);

                    builder.addDoubleProperty(
                            "Back Left Velocity", () -> moduleStates[3].speedMetersPerSecond, null);

                    builder.addDoubleProperty("Robot Angle", () -> rotationDegrees, null);
                }
            };

    @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
    private final Field2d fieldVis = new Field2d();

    private final Alert overcurrent = new Alert("General overcurrent!", AlertType.WARNING);

    private final Alert highResistance =
            new Alert("High main power path resistance!", AlertType.WARNING);

    private final Alert lowVoltage = new Alert("Low battery voltage!", AlertType.WARNING);

    public UI() {
        if (!Robot.isReal()) new Alert("Robot is simulated!", AlertType.INFO).set(true);
        if (DriverStation.isFMSAttached()) new Alert("FMS connected!", AlertType.INFO).set(true);
    }

    public void setSwerveStates(SwerveModuleState[] states) {
        moduleStates = states;
    }

    public void setPose(Pose2d pose) {
        fieldVis.setRobotPose(pose);

        rotationDegrees =
                pose.getRotation().getDegrees(); // for the sake of swerve viz getting an angle
    }

    public void setSetpointPose(Pose2d setpointPose) {
        fieldVis.getObject("setpointPose").setPose(setpointPose);
    }

    public void setBatteryVoltage(double volts) {
        log("batteryVoltage", volts);

        voltage = volts;

        lowVoltage.set(volts < Constants.AlertConstants.LOW_VOLTAGE_THRESHOLD);

        logPowerPathResistance();
    }

    public void setTotalCurrentDraw(double amps) {
        log("totalCurrent", amps);

        currentAmps = amps;

        overcurrent.set(amps > Constants.AlertConstants.OVERCURRENT_THRESHOLD_AMPS);

        logPowerPathResistance();
    }

    private void logPowerPathResistance() {
        double resistance;

        if (currentAmps == 0) resistance = 0;
        else resistance = voltage / currentAmps;

        log("mainPowerPathResistanceOhms", resistance);

        highResistance.set(resistance > Constants.AlertConstants.HIGH_RESISTANCE_THRESHOLD_OHMS);
    }
}
