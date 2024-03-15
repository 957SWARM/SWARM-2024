package com.team957.comp2024.util;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.team957.comp2024.Constants.SparkMaxAlertsConstants;
import com.team957.comp2024.util.FaultDetectionUtil.SignalJumpDetector;
import com.team957.comp2024.util.FaultDetectionUtil.StaleNoisyDataDetector;
import java.util.ArrayList;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class SparkMaxUtils {
    // object allocation bad!
    private static final ArrayList<String> faultsBuffer = new ArrayList<>();

    public static String[] getFaultsAsStringArray(CANSparkMax sMax, boolean sticky) {
        faultsBuffer.clear();

        for (FaultID fault : FaultID.values()) {
            if ((sticky) ? sMax.getStickyFault(fault) : sMax.getFault(fault))
                faultsBuffer.add(fault.toString());
        }

        return faultsBuffer.toArray(new String[] {});
    }

    /**
     * Can't Spark Max, if you will
     *
     * <p>Mutates & returns the given SparkMax with all selected periodic frames (indicated with
     * `true`) slowed down
     */
    public static CANSparkMax slowUnusedPeriodics(
            CANSparkMax sparkMax,
            boolean rotorPosition,
            boolean analogEncoder,
            boolean alternateEncoder,
            boolean dutyCyclePosition,
            boolean dutyCycleVelocity) {
        int normal = 20; // milliseconds
        int slow = 1000; // milliseconds

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, rotorPosition ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, analogEncoder ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, alternateEncoder ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, dutyCyclePosition ? slow : normal);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, dutyCycleVelocity ? slow : normal);

        return sparkMax;
    }

    public static class SparkMaxAlertsUtil {
        private final CANSparkMax sMax;

        private final Alert overTemp;
        private final Alert overCurrent;
        private final Alert sensorFault;
        private final Alert motorFault;
        private final Alert canFault;

        private final StaleNoisyDataDetector stalenessDetector;

        private final SignalJumpDetector jumpDetector;

        private final double currentLimitAmps;

        public SparkMaxAlertsUtil(CANSparkMax sMax, String name, double currentLimitAmps) {
            this.sMax = sMax;

            this.overTemp = new Alert("SparkMax " + name + " overtemperature!", AlertType.WARNING);

            this.overCurrent = new Alert("SparkMax " + name + " overcurrent!", AlertType.WARNING);

            this.sensorFault = new Alert("SparkMax " + name + " sensor fault!", AlertType.ERROR);

            this.motorFault = new Alert("SparkMax " + name + " motor fault!", AlertType.ERROR);

            this.canFault = new Alert("SparkMax " + name + " can fault!", AlertType.ERROR);

            stalenessDetector =
                    new StaleNoisyDataDetector(
                            "SparkMax " + name + " voltage measurement",
                            SparkMaxAlertsConstants.VOLTAGE_MEASUREMENT_STALENESS_THRESHOLD);

            jumpDetector =
                    new SignalJumpDetector(
                            SparkMaxAlertsConstants.SUSPICIOUS_TEMPERATURE_RATE,
                            false,
                            "SparkMax " + name + " temperature measurement");

            this.currentLimitAmps = currentLimitAmps;
        }

        public void poll() {
            overTemp.set(
                    sMax.getMotorTemperature()
                            > SparkMaxAlertsConstants.OVERTEMPERATURE_THRESHOLD_C);
            overCurrent.set(sMax.getOutputCurrent() > currentLimitAmps);
            sensorFault.set(sMax.getFault(FaultID.kSensorFault));
            motorFault.set(sMax.getFault(FaultID.kMotorFault));
            canFault.set(sMax.getFault(FaultID.kCANRX) || sMax.getFault(FaultID.kCANTX));

            stalenessDetector.poll(sMax.getBusVoltage());
            jumpDetector.poll(sMax.getMotorTemperature());
        }
    }
}
