package com.team957.comp2024.subsystems.swerve;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.lib.math.UtilityMath;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveSim extends Swerve {
    private static class SimModuleIO extends ModuleIO {
        final DCMotorSim steerSim =
                new DCMotorSim(
                        SwerveConstants.STEER_MOTOR,
                        SwerveConstants.STEER_GEARING,
                        SwerveConstants.STEER_MOMENT_KG_M_2);

        final FlywheelSim driveSim =
                new FlywheelSim(
                        SwerveConstants.DRIVE_MOTOR,
                        SwerveConstants.DRIVE_GEARING,
                        SwerveConstants.DRIVE_MOMENT_KG_M_2);

        // dc motor sim supports position, flywheel does not

        double steerInputVoltage = 0;
        double driveInputVoltage = 0;

        @Override
        public void setSteerSetpoint(double radians) {
            steerSim.setState(radians, 0);
            // directly setting state, no actual controller in the loop
            // womp womp for no revlib simulation support

            steerOnboardControl = true;
        }

        @Override
        public void setSteerVoltage(double volts) {
            steerSim.setInputVoltage(volts);

            steerOnboardControl = false;
            steerInputVoltage = volts;
        }

        @Override
        public double getSteerControlEffortVolts() {
            return steerInputVoltage;
        }

        @Override
        public double getSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(steerSim.getAngularPositionRad());
        }

        @Override
        public double getSteerCurrentAmps() {
            return steerSim.getCurrentDrawAmps();
        }

        @Override
        public void setDriveSetpoint(double radPerSecond) {
            driveSim.setState(radPerSecond);
            // :( rev

            steerOnboardControl = true;
        }

        @Override
        public void setDriveVoltage(double volts) {
            driveSim.setInputVoltage(volts);

            driveOnboardControl = false;
            driveInputVoltage = volts;
        }

        @Override
        public double getDriveControlEffortVolts() {
            return driveInputVoltage;
        }

        @Override
        public double getDriveVelocityRadPerSecond() {
            return driveSim.getAngularVelocityRadPerSec();
        }

        @Override
        public double getDriveCurrentAmps() {
            return driveSim.getCurrentDrawAmps();
        }

        private void updateSims(double dt) {
            steerSim.update(dt);
            driveSim.update(dt);
        }
    }

    private final SimModuleIO frontLeft = new SimModuleIO();
    private final SimModuleIO frontRight = new SimModuleIO();
    private final SimModuleIO backRight = new SimModuleIO();
    private final SimModuleIO backLeft = new SimModuleIO();

    private final DeltaTimeUtil dtUtil = new DeltaTimeUtil();

    @Override
    public ModuleIO getFrontLeft() {
        return frontLeft;
    }

    @Override
    public ModuleIO getFrontRight() {
        return frontRight;
    }

    @Override
    public ModuleIO getBackRight() {
        return backRight;
    }

    @Override
    public ModuleIO getBackLeft() {
        return backLeft;
    }

    @Override
    public void periodic() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        frontLeft.updateSims(dt);
        frontRight.updateSims(dt);
        backRight.updateSims(dt);
        backLeft.updateSims(dt);
    }
}
