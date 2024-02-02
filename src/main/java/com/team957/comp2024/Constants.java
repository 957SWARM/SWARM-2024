package com.team957.comp2024;

import com.team957.lib.controllers.feedback.PID.PIDConstants;
import com.team957.lib.util.GearRatioHelper;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class MiscConstants {
        public static final double saturationVoltage = 12;
    }

    public static final class SwerveConstants {
        public static final double DRIVE_CURRENT_LIMIT = 40;
        public static final double STEER_CURRENT_LIMIT = 20;

        public static final boolean INITIAL_BRAKE_MODE_ACTIVE = false;
        // for the sake of the people positioning the robot!!

        public static final double STEER_PLANT_KS = 0;
        public static final double STEER_PLANT_KV = 0.1; // wild guess
        public static final double STEER_PLANT_KA = 0.1;

        public static final double DRIVE_PLANT_KS = 0;
        public static final double DRIVE_PLANT_KV = 0.1;
        public static final double DRIVE_PLANT_KA = 0.1;

        public static final double STEER_GEARING = 42.62;
        public static final double DRIVE_GEARING = 4.71;

        public static final GearRatioHelper DRIVE_GEARING_HELPER =
                new GearRatioHelper(1, DRIVE_GEARING);

        public static final DCMotor STEER_MOTOR = DCMotor.getNeo550(1);
        public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        public static final double MAX_WHEEL_SPEED_METERS_PER_SECOND = 4;

        public static final Translation2d FRONT_LEFT_TRANSLATION =
                new Translation2d(0.3112, 0.2604);
        public static final Translation2d FRONT_RIGHT_TRANSLATION =
                new Translation2d(0.3112, -0.2604);
        public static final Translation2d BACK_RIGHT_TRANSLATION =
                new Translation2d(-0.3112, -0.2604);
        public static final Translation2d BACK_LEFT_TRANSLATION =
                new Translation2d(-0.3112, 0.2604);

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        FRONT_LEFT_TRANSLATION,
                        FRONT_RIGHT_TRANSLATION,
                        BACK_RIGHT_TRANSLATION,
                        BACK_LEFT_TRANSLATION);

        public static final double ONBOARD_DRIVE_CONTROLLER_KP = 0.00003;
        public static final double ONBOARD_DRIVE_CONTROLLER_KI = 0;
        public static final double ONBOARD_DRIVE_CONTROLLER_KD = 0;
        public static final double ONBOARD_DRIVE_CONTROLLER_KFF = 0.000125;

        public static final double ONBOARD_STEER_CONTROLLER_KP = 2;
        public static final double ONBOARD_STEER_CONTROLLER_KI = 0;
        public static final double ONBOARD_STEER_CONTROLLER_KD = 0;

        // the offset is the "raw" value reported when the module is at the "zero" position

        public static final int FRONT_LEFT_DRIVE_CANID = 7;
        public static final int FRONT_LEFT_STEER_CANID = 8;
        public static final double FRONT_LEFT_STEER_OFFSET_RADIANS = -0.3613 + (Math.PI / 2);
        public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;

        public static final int FRONT_RIGHT_DRIVE_CANID = 1;
        public static final int FRONT_RIGHT_STEER_CANID = 2;
        public static final double FRONT_RIGHT_STEER_OFFSET_RADIANS = -1.5327;
        public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;

        public static final int BACK_LEFT_DRIVE_CANID = 5;
        public static final int BACK_LEFT_STEER_CANID = 6;
        public static final double BACK_LEFT_STEER_OFFSET_RADIANS = -0.0124;
        public static final boolean BACK_LEFT_DRIVE_INVERTED = false;

        public static final int BACK_RIGHT_DRIVE_CANID = 3;
        public static final int BACK_RIGHT_STEER_CANID = 4;
        public static final double BACK_RIGHT_STEER_OFFSET_RADIANS = -5.6515 + (Math.PI / 2);
        public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;
    }

    public static final class ShooterConstants {
        // CANIDs not set yet as of 1/25/2024
        public static final int LEFT_CANID = 0;
        public static final int RIGHT_CANID = 0;

        public static final int CURRENT_LIMIT = 30;
        public static final DCMotor SHOOTER_MOTOR = DCMotor.getNEO(1);
        public static final double SHOOTER_REDUCTION = 1;
        public static final double DEFAULT_VOLTAGE = 0;
        public static final double SHOOTING_VOLTAGE = 11.0;

        public static final boolean leftMotorInverted = false;
        public static final boolean rightMotorInverted = true;
        public static final boolean leftEncoderInverted = false;
        public static final boolean rightEncoderInverted = true;
    }

    public static final class AutoConstants {
        public static final PIDConstants LINEAR_PATHFINDING_GAINS = new PIDConstants(5, 0, 0);
        public static final PIDConstants ROTATIONAL_PATHFINDING_GAINS = new PIDConstants(5, 0, 0);
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

    public static final class IntakePivotConstants {
        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(-10);
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(135);

        public static final double PLANT_KS = 1;
        public static final double PLANT_KV = 0.88;
        public static final double PLANT_KA = 0.001;
        public static final double PLANT_KG = 0.22;

        public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

        public static final GearRatioHelper GEARING_HELPER = new GearRatioHelper(1, 45);

        public static final double PIVOT_TO_TIP_METERS = Units.inchesToMeters(14);
    }

    public static final class IntakeRollerConstants {
        // CAN ID not set as of 2/1/2024
        public static final int ROLLER_CANID = 0;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ROLLER_INVERTED = false;

        public static final double INTAKE_VOLTAGE = 6;
        public static final double PUKE_VOLTAGE = -INTAKE_VOLTAGE;
    }

    public static final class AlertConstants {
        public static final double OVERCURRENT_THRESHOLD_AMPS = 120;

        public static final double HIGH_RESISTANCE_THRESHOLD_OHMS = 20.0 / 1000; // 20 mOhms

        public static final double LOW_VOLTAGE_THRESHOLD = 10.5;
    }
}
