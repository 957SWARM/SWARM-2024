package com.team957.comp2024.util;

import com.team957.lib.math.UtilityMath;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;

public class FaultDetectionUtil {
    public static class LingeringAlert {
        private final Alert alert;

        private double baseTime = -957957957;
        // start below zero so we don't boot into a lingering state
        // where the timer is at zero not because of the conditional being true
        // will technically break if someone tries to set a linger time of 30+ years

        private final String text;
        private final double lingerTimeSeconds;
        private final String textWithAsterisk;

        public LingeringAlert(String text, AlertType type, double lingerTimeSeconds) {
            alert = new Alert(text, type);

            this.text = text;
            this.lingerTimeSeconds = lingerTimeSeconds;
            textWithAsterisk = "*" + text;
        }

        public void set(boolean condition) {
            if (condition) {
                alert.setText(text);
                alert.set(true);

                baseTime = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - lingerTimeSeconds < baseTime) {
                alert.setText(textWithAsterisk);
                alert.set(true);
            } else {
                alert.set(false);
            }
        }
    }

    public static class StaleNoisyDataDetector {
        private final LingeringAlert alert;

        private final int repeatThreshold;

        private double lastValue = 0;
        private boolean isFirstRun = true;

        private int numRepeats = 0;

        public StaleNoisyDataDetector(String name, int repeatThreshold) {
            alert = new LingeringAlert(name + " may be stale!", AlertType.WARNING, 2);

            this.repeatThreshold = repeatThreshold;
        }

        public boolean poll(double value) {
            boolean result;

            if (isFirstRun) {
                isFirstRun = false;
                result = false;
            } else {
                if (lastValue == value) numRepeats++;
                else numRepeats = 0;

                result = (numRepeats > repeatThreshold);
            }

            alert.set(result);

            return result;
        }
    }

    public static class SignalJumpDetector {
        private final LingeringAlert jumpDetected;

        private final double suspiciousDerivativeThreshold;

        private boolean isFirstRun = true;

        private final AngularDifferentiator finiteDifference;

        public SignalJumpDetector(double suspiciousRateThreshold, boolean angular, String name) {
            finiteDifference = new AngularDifferentiator(angular);

            jumpDetected =
                    new LingeringAlert(
                            "Potential jump detected in signal " + name + "!",
                            AlertType.WARNING,
                            5);

            this.suspiciousDerivativeThreshold = suspiciousRateThreshold;
        }

        public boolean poll(double value) {
            if (isFirstRun) {
                finiteDifference.calculate(value);

                isFirstRun = false;

                return false;
            } else {
                boolean jumped =
                        Math.abs(finiteDifference.calculate(value)) > suspiciousDerivativeThreshold;

                jumpDetected.set(jumped);

                return jumped;
            }
        }
    }

    // will be upstreamed into the library at some point
    private static class AngularDifferentiator {
        final boolean angular;

        final DeltaTimeUtil dtUtil = new DeltaTimeUtil();

        double lastValue = 0;

        AngularDifferentiator(boolean angular) {
            this.angular = angular;
        }

        // expects radians for angular values
        double calculate(double value) {
            double dt = dtUtil.getTimeSecondsSinceLastCall();

            if (dt == 0) return 0;
            double out;

            if (!angular) out = (value - lastValue) / dt;
            else out = UtilityMath.smallestAngleRadiansBetween(value, lastValue) / dt;

            lastValue = value;
            return out;
        }
    }
}
