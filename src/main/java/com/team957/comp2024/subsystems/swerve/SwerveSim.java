package com.team957.comp2024.subsystems.swerve;

import com.team957.comp2024.Constants.SwerveConstants;
import com.team957.lib.math.UtilityMath;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveSim extends Swerve {
    private static class SimModuleIO extends ModuleIO {
        final DCMotorSim steerSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                SwerveConstants.STEER_PLANT_KV, SwerveConstants.STEER_PLANT_KA),
                        SwerveConstants.STEER_MOTOR,
                        SwerveConstants.STEER_GEARING);

        final DCMotorSim driveSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                SwerveConstants.DRIVE_PLANT_KV, SwerveConstants.DRIVE_PLANT_KA),
                        SwerveConstants.DRIVE_MOTOR,
                        SwerveConstants.DRIVE_GEARING);

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
            driveSim.setState(driveSim.getAngularPositionRad(), radPerSecond);
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

        public double getDrivePositionRad() {
            return driveSim.getAngularPositionRad();
        }

        @Override
        public double getDriveCurrentAmps() {
            return driveSim.getCurrentDrawAmps();
        }

        @Override
        protected void update(double dt) {
            steerSim.update(dt);
            driveSim.update(dt);
        }
    }

    private final DeltaTimeUtil dtUtil = new DeltaTimeUtil();

    public SwerveSim() {
        super(new SimModuleIO(), new SimModuleIO(), new SimModuleIO(), new SimModuleIO());
    }

    @Override
    public void periodic() {
        super.periodic();

        double dt = dtUtil.getTimeSecondsSinceLastCall();

        frontLeft.update(dt);
        frontRight.update(dt);
        backRight.update(dt);
        backLeft.update(dt);
    }
}
