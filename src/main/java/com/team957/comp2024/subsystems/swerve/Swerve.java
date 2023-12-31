package com.team957.comp2024.subsystems.swerve;

import com.team957.comp2024.Constants.SwerveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public abstract class Swerve implements Subsystem, Logged {
    public abstract static class ModuleIO implements Logged {
        protected ModuleIO() {}

        // starting values of these does not effect how the module is controlled
        // just what "*IsOnboardControlLoop() returns before "set*()" is called
        protected boolean steerOnboardControl = false;
        protected boolean driveOnboardControl = false;

        /**
         * Sets the setpoint for the onboard controller. If none is present (running in simulation),
         * instantly snaps the steer to its setpoint.
         *
         * <p>If steer is currently not running off of onboard closed-loop, switches to that mode.
         *
         * @param radians Setpoint for the azimuth, in radians.
         */
        public abstract void setSteerSetpoint(double radians);

        /**
         * Sets the applied voltage to the steer motor.
         *
         * <p>If steer is currently not running off of voltage control mode, switches to that mode.
         *
         * @param radians Voltage to be applied.
         */
        public abstract void setSteerVoltage(double volts);

        /**
         * Gets the voltage applied, either directly or through a controller, to the steer motor.
         *
         * <p>Does not work with controllers in simulation.
         *
         * @return The set steer voltage.
         */
        @LogBoth
        public abstract double getSteerControlEffortVolts();

        @LogBoth
        public boolean steerIsOnboardClosedLoop() {
            return steerOnboardControl;
        }

        @LogBoth
        public abstract double getSteerPositionRadians();

        @LogBoth
        public abstract double getSteerCurrentAmps();

        /**
         * Sets the setpoint for the onboard controller. If none is present (running in simulation),
         * instantly snaps the drive to its setpoint.
         *
         * <p>If drive is currently not running off of onboard closed-loop, switches to that mode.
         *
         * @param radians Setpoint for the drive, in radians per second.
         */
        public abstract void setDriveSetpoint(double radPerSecond);

        /**
         * Sets the setpoint for the onboard controller, using units of meters per second. If none
         * is present (running in simulation), instantly snaps the drive to its setpoint.
         *
         * <p>If drive is currently not running off of onboard closed-loop, switches to that mode.
         *
         * @param radians Setpoint for the drive, in meters per second.
         */
        public void setDriveSetpointMPS(double metersPerSecond) {
            setDriveSetpoint(metersPerSecond / (2 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS));
        }

        /**
         * Sets the applied voltage to the drive motor.
         *
         * <p>If drive is currently not running off of voltage control mode, switches to that mode.
         *
         * @param radians Voltage to be applied.
         */
        public abstract void setDriveVoltage(double volts);

        /**
         * Gets the voltage applied, either directly or through a controller, to the drive motor.
         *
         * <p>Does not work with controllers in simulation.
         *
         * @return The set drive voltage.
         */
        @LogBoth
        public abstract double getDriveControlEffortVolts();

        @LogBoth
        public boolean driveIsOnboardClosedLoop() {
            return driveOnboardControl;
        }

        @LogBoth
        public abstract double getDriveVelocityRadPerSecond();

        public double getDriveVelocityMetersPerSecond() {
            return 2
                    * Math.PI
                    * getDriveVelocityRadPerSecond()
                    * SwerveConstants.WHEEL_RADIUS_METERS;
        }

        @LogBoth
        public abstract double getDriveCurrentAmps();

        // object allocations :(
        @LogBoth
        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    getDriveVelocityMetersPerSecond(),
                    Rotation2d.fromRadians(getSteerPositionRadians()));
        }
    }

    public static Swerve getSwerve(boolean isReal) {
        return (isReal) ? new SwerveHW() : new SwerveSim();
    }

    public abstract ModuleIO getFrontLeft();

    public abstract ModuleIO getFrontRight();

    public abstract ModuleIO getBackRight();

    public abstract ModuleIO getBackLeft();
}
