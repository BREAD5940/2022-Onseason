package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;

// All Constants
public final class Constants {

    // Constants pertaining to the drive subsystem go here
    public static class Drive {

        // Motor IDs 
        public static final int[] DRIVE_IDS = {1, 2, 3, 4};
        public static final int[] STEER_IDS = {5, 6, 7, 8};
        public static final int HORIZONTAL_GUT = 10;
        public static final int VERTICAL_GUT = 11;

        // Encoder Channels
        public static final int[] AZIMUTH_CHANNELS = {21, 22, 23, 24};

        // Reversed Constants
        public static final boolean[] DRIVES_ARE_REVERSED = {false, true, false, true};
        public static final boolean[] STEERS_ARE_REVERSED = {true, true, true, true};
        public static final boolean[] AZIMUTHS_ARE_REVERSED = {false, false, false, false};

        // Offsets (calculate offsets by measuring the values shown in the pheonix tuner self-test snapshot when all offsets are set to 0.0)
        public static final Rotation2d[] AZIMUTH_OFFSETS = {
            Rotation2d.fromDegrees(-107.139), // FL
            Rotation2d.fromDegrees(54.492), // FR
            Rotation2d.fromDegrees(-58.535), // BL
            Rotation2d.fromDegrees(-47.725) //BR
        };

        // Gearings
        public static final double MODULE_GEARING = (14.0/50.0) * (28.0/16.0) * (15.0/45.0);
        
        // Constant measurements
        public static final double ROBOT_WIDTH = Units.inchesToMeters(25.0);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(30.0);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0) * 0.9442667069;
        public static final Translation2d FIELD_TO_TARGET = new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13.5));
        public static final double ROBOT_MAX_SPEED = (6380.0 * MODULE_GEARING * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        public static final Translation2d FL_LOCATION = new Translation2d(ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        public static final Translation2d FR_LOCATION = new Translation2d(ROBOT_LENGTH/2, -ROBOT_WIDTH/2);
        public static final Translation2d BL_LOCATION = new Translation2d(-ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        public static final Translation2d BR_LOCATION = new Translation2d(-ROBOT_LENGTH/2, -ROBOT_WIDTH/2); 
    
        // Other
        public static final double CANCODER_RESOLUTION = 4096.0;

    }

    // Constant pertaining to the gut subsystem go here
    public static class Gut {

        // Motor IDs 
        public static int GUT_ID = 11;
        
        // Sensor Channels
        public static final int leftBeamBreakChannel = 0;
        public static final int middleBeamBreakChannel = 1;
        public static final int rightBeamBreakChannel = 2;
        public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;

    }   

}
