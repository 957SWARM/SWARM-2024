package com.team957.comp2024.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team957.comp2024.util.SparkMaxUtils;
import monologue.Monologue.LogBoth;

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
        public boolean driveIsOnboardClosedLoop() {
            return false;
        }

        @Override
        public double getDriveVelocityRadPerSecond() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getDriveVelocityMetersPerSecond'");
        }

        @Override
        public double getDriveCurrentAmps() {
            return drive.getOutputCurrent();
        }

        @LogBoth
        public double getSteerTemperatureC() {
            return steer.getMotorTemperature();
        }

        @LogBoth
        public double getDriveTemperatureC() {
            return drive.getMotorTemperature();
        }

        @LogBoth
        public double getSteerBusVoltage() {
            return steer.getBusVoltage();
        }

        @LogBoth
        public double getDriveBusVoltage() {
            return drive.getBusVoltage();
        }

        @LogBoth
        public double getUnoffsetSteerPositionRadians() {
            return 0;
        }

        @LogBoth
        public String[] getSteerStickyFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(steer, true);
        }

        @LogBoth
        public String[] getSteerFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(steer, false);
        }

        @LogBoth
        public String[] getDriveStickyFaults() {
            return SparkMaxUtils.getFaultsAsStringArray(drive, true);
        }

        @LogBoth
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
