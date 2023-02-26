package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// All Constants
public final class Constants {

    // Constants pertaining to the drive subsystem go here
    public static class Drive {

        // Motor IDs
        public static final int[] DRIVE_IDS = { 1, 2, 3, 4 };
        public static final int[] STEER_IDS = { 5, 6, 7, 8 };
        public static final int PIGEON_ID = 30;

        // Encoder Channels
        public static final int[] AZIMUTH_CHANNELS = { 21, 22, 23, 24 };

        // Reversed Constants
        public static final TalonFXInvertType[] DRIVE_INVERT_TYPES = {
                TalonFXInvertType.CounterClockwise,
                TalonFXInvertType.Clockwise,
                TalonFXInvertType.CounterClockwise,
                TalonFXInvertType.Clockwise
        };
        public static final boolean[] STEERS_ARE_REVERSED = { true, true, true, true };
        public static final boolean[] AZIMUTHS_ARE_REVERSED = { false, false, false, false };

        // Offsets (calculate offsets by measuring the values shown in the pheonix tuner
        // self-test snapshot when all offsets are set to 0.0)
        // Offsets change on boot
        public static final Rotation2d[] AZIMUTH_OFFSETS = {
                Rotation2d.fromDegrees(-106.12 - 33.223), // FL
                Rotation2d.fromDegrees(54.053 + 0.527), // FR
                Rotation2d.fromDegrees(-57.74 - 0.967), // BL
                Rotation2d.fromDegrees(-41.748 - 0.176) // BR
        };

        // Measurements/Gearings
        public static final double MODULE_GEARING = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(27.0);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(28.0);
        // Madtown field callibration constant factor is 0.97
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0) * 0.9442667069;
        public static final Translation2d FIELD_TO_TARGET = new Translation2d(Units.feetToMeters(27),
                Units.feetToMeters(13.5));
        public static final double CAMERA_TO_SHOOTER_DISTANCE = Units.inchesToMeters(15.0);
        public static final double UPPER_HUB_RADIUS = Units.inchesToMeters(53.38) / 2;
        public static final double ROBOT_MAX_SPEED = (6380.0 * MODULE_GEARING * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        public static final Translation2d FL_LOCATION = new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
        public static final Translation2d FR_LOCATION = new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
        public static final Translation2d BL_LOCATION = new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
        public static final Translation2d BR_LOCATION = new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);

        // Other
        public static final double CANCODER_RESOLUTION = 4096.0;
    }

    // Constants pertaining to electrical
    public static class Electrical {
        public static final String CANIVORE_BUS_NAME = "dabus";
    }
}
