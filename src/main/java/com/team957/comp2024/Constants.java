package com.team957.comp2024;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class MiscConstants {
        public static final double saturationVoltage = 12;
    }

    public static final class SwerveConstants {
        public static final double STEER_PLANT_KS = 1;
        public static final double STEER_PLANT_KV = 1; // wild guess
        public static final double STEER_PLANT_KA = 1;

        public static final double DRIVE_PLANT_KS = 1;
        public static final double DRIVE_PLANT_KV = 1;
        public static final double DRIVE_PLANT_KA = 1;

        public static final double STEER_GEARING = 42.62;
        public static final double DRIVE_GEARING = 4.71;

        public static final DCMotor STEER_MOTOR = DCMotor.getNeo550(1);
        public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3);

        public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.5, 0.5);
        public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.5, -0.5);
        public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-0.5, -0.5);
        public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-0.5, 0.5);

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        FRONT_LEFT_TRANSLATION,
                        FRONT_RIGHT_TRANSLATION,
                        BACK_RIGHT_TRANSLATION,
                        BACK_LEFT_TRANSLATION);
    }

    public static final class IMUConstants {
        public static final boolean YAW_AXIS_INVERTED = true;
        // this should NOT be applied to the navx.getRotation2D(), that's already offset
        // this is for the angular rate

        // the value reported as "unoffset" when the robot is zeroed
        public static final double YAW_OFFSET = -Math.PI;
    }

    public static final class PDHConstants {
        public static final int PDH_CAN_ID = 30;

        public static final boolean STARTING_SWITCHABLE_CHANNEL_STATE = false;
    }

    public static final class PneumaticHubConstants {
        public static final int PH_CAN_ID = 31;

        public static final int PRESSURE_SENSOR_ANALOG_CHANNEL = 0;
    }
}
