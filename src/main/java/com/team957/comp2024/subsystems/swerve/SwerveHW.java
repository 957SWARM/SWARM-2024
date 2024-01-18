package com.team957.comp2024.subsystems.swerve;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team957.comp2024.util.SparkMaxUtils;
import monologue.Annotations.Log;

public class SwerveHW extends Swerve {

    private static class HWModuleIO extends ModuleIO {
        private final CANSparkMax steer;
        private final CANSparkMax drive;

        HWModuleIO(int steerCANID, int driveCANID, double steerOffsetRadians) {
            steer = new CANSparkMax(steerCANID, MotorType.kBrushless);
            drive = new CANSparkMax(driveCANID, MotorType.kBrushless);
        }

        @Override
        public void setSteerSetpoint(double radians) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setSteerSetpoint'");
        }

        @Override
        public void setSteerVoltage(double volts) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setSteerVoltage'");
        }

        @Override
        public double getSteerControlEffortVolts() {
            return steer.getAppliedOutput() * getSteerBusVoltage();
        }

        @Override
        public boolean steerIsOnboardClosedLoop() {
            return false;
        }

        @Override
        public double getSteerPositionRadians() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getSteerPositionRadians'");
        }

        @Override
        public double getSteerCurrentAmps() {
            return steer.getOutputCurrent();
        }

        @Override
        public void setDriveSetpoint(double radPerSecond) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setDriveSetpoint'");
        }

        @Override
        public void setDriveVoltage(double volts) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setDriveVoltage'");
        }

        @Override
        public double getDriveControlEffortVolts() {
            return drive.getAppliedOutput() * getDriveBusVoltage();
        }

        @Override
        @Log
        public boolean driveIsOnboardClosedLoop() {
            return false;
        }

        @Override
        @Log
        public double getDriveVelocityRadPerSecond() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getDriveVelocityMetersPerSecond'");
        }

        @Log
        public double getDrivePositionRad() {
            return 0;
        }

        @Override
        @Log
        public double getDriveCurrentAmps() {
            return drive.getOutputCurrent();
        }

        @Log
        public double getSteerTemperatureC() {
            return steer.getMotorTemperature();
        }

        @Log
        public double getDriveTemperatureC() {
            return drive.getMotorTemperature();
        }

        @Log
        public double getSteerBusVoltage() {
            return steer.getBusVoltage();
        }

        @Log
        public double getDriveBusVoltage() {
            return drive.getBusVoltage();
        }

        @Log
        public double getUnoffsetSteerPositionRadians() {
            return 0;
        }

        @Log
        public String[] getSteerStickyFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(steer, true);
        }

        @Log
        public String[] getSteerFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(steer, false);
        }

        @Log
        public String[] getDriveStickyFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(drive, true);
        }

        @Log
        public String[] getDriveFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(drive, false);
        }

        @Override
        protected void update(double dt) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'update'");
        }
    }

    public SwerveHW() {
        super(null, null, null, null);
    }
}
