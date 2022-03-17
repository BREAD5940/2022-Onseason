package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

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
        public static final TalonFXInvertType[] DRIVE_INVERT_TYPES = {
            TalonFXInvertType.CounterClockwise, 
            TalonFXInvertType.Clockwise, 
            TalonFXInvertType.CounterClockwise, 
            TalonFXInvertType.Clockwise
        };
        public static final boolean[] STEERS_ARE_REVERSED = {true, true, true, true};
        public static final boolean[] AZIMUTHS_ARE_REVERSED = {false, false, false, false};

        // Offsets (calculate offsets by measuring the values shown in the pheonix tuner self-test snapshot when all offsets are set to 0.0)
        // Offsers change on boot
        public static final Rotation2d[] AZIMUTH_OFFSETS = {
            Rotation2d.fromDegrees(-106.12), // FL
            Rotation2d.fromDegrees(54.053), // FR
            Rotation2d.fromDegrees(-57.744), // BL
            Rotation2d.fromDegrees(-41.748) //BR
        };

        // Measurements/Gearings
        public static final double MODULE_GEARING = (14.0/50.0) * (28.0/16.0) * (15.0/45.0);
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
        public static final int GUT_ID = 11;
        
        // Sensor Channels
        public static final int LEFT_BEAM_BREAK_CHANNEL = 3;
        public static final int MIDDLE_BEAM_BREAK_CHANNEL = 1;
        public static final int RIGHT_BEAM_BREAK_CHANNEL = 2;
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kMXP;

        // Measurements/Gearings
        public static final double GUT_GEARING = (24.0/36.0);
        public static final double GUT_PULLEY_DIAMETER = Units.inchesToMeters(1.0);
        public static final double MAX_GUT_FALCON_SHAFT_SPEED = 5366.082442; // RPM
        public static final double MAX_GUT_SURFACE_SPEED = (MAX_GUT_FALCON_SHAFT_SPEED * GUT_GEARING * Math.PI * GUT_PULLEY_DIAMETER)/60.0;

        // Color Sensor Targets
        public static final Color COLOR_SENSOR_RED_TARGET = new Color(0.5634765625, 0.320556640625, 0.1166992187);
        public static final Color COLOR_SENSOR_BLUE_TARGET = new Color(0.134521484375, 0.370849609375, 0.4951171875);
        public static final Color COLOR_SENSOR_NO_TARGET = new Color(0.2578125, 0.464111328125, 0.464111328125);

    }   

    // Constants pertaining to the dual intake subsystem go here
    public static class DualIntake {

        // Motor IDs 
        public static final int LEFT_INTAKE_ID = 9;
        public static final int RIGHT_INTAKE_ID = 10;

        // Intake Piston Channels
        public static final int[] LEFT_INTAKE_PISTON_CHANNELS = {6, 1}; // [Forward, Reverse]
        public static final int[] RIGHT_INTAKE_PISTON_CHANNELS = {7, 0}; // [Forward, Reverse]

        // Measurements
        public static final double INTAKE_SPEED = 0.8;
        public static final double SPIT_SPEED = 0.3;

    }

    // Constants pertaining to the flywheel subsystem go here
    public static class Flywheel {

        // Motor IDs
        public static final int LEFT_MOTOR_ID = 13;
        public static final int RIGHT_MOTOR_ID = 14;

        // Measurements/Gearings
        public static final double MAX_FLYWHEEL_RPM = 6163.0;
        public static final double SHOOTER_IDLE_VEL = 0.0;

        // Other
        public static final TalonFXInvertType RIGHT_MOTOR_DIRECTION = TalonFXInvertType.OpposeMaster;
        public static final TalonFXInvertType LEFT_MOTOR_DIRECTION = TalonFXInvertType.CounterClockwise;
    }

    // Constants pertaining to the neck subsystem go here
    public static class Neck {

        // Motor IDs
        public static final int NECK_ID = 12;

        // Sensor Channels
        public static final int TOP_BEAM_BREAK_CHANNEL = 0;
        
        // Measurements/Gearings
        public static final double NECK_GEARING = (24.0/36.0);
        public static final double NECK_PULLEY_DIAMETER = Units.inchesToMeters(0.975);
        public static final double MAX_NECK_FALCON_SHAFT_SPEED = 4963.250411;
        public static final double MAX_NECK_SURFACE_SPEED = (MAX_NECK_FALCON_SHAFT_SPEED * NECK_GEARING * Math.PI * NECK_PULLEY_DIAMETER)/60.0;
    }

    // Constants pertaining to the hood subsystem go here
    public static class Hood {

        // Motor IDs and Other Channels
        public static final int HOOD_MOTOR_ID = 15;

        // Constants/Measurements
        public static final int COUNTS_PER_REVOLUTION = 8192;
        public static final double HOOD_GEARING = (20.0/460.0);
        public static final double HOOD_IDLE_POS = 8.5;
        public static final double MIN_HOOD_TRAVEL = 0.0;
        public static final double MAX_HOOD_TRAVEL = 37.928627014160156; 

    }

    // Constants pertaining the the climber subsystem go here
    public static class Climber {

        // Motor IDs
        public static final int TOP_CLIMBER_MOTOR_ID = 16;
        public static final int BOTTOM_CLIMBER_MOTOR_ID = 17;
        public static final int CLIMBER_FORWARD_CHANNEL = 2;
        public static final int CLIMBER_REVERSE_CHANNEL = 5;

        // Measurements/Constants
        public static final double CLIMBER_GEARING = 1.0/12.86;
        public static final double CLIMBER_PITCH_DIAMETER = Units.inchesToMeters(2.256);
        public static final double CLIMBER_MINIMUM_TRAVEL = 0;
        public static final double CLIMBER_RETRACTED_HEIGHT = 0.00025;
        public static final double CLIMBER_MID_RUNG_HEIGHT = 0.58;
        public static final double CLIMBER_HEIGHT_BEFORE_NEXT_RUNG = 0.3;
        public static final double CLIMBER_HEIGHT_TRANSITIONING_TO_NEXT_RUNG = 0.61;
        public static final double CLIMBER_HEIGHT_PULLED_OFF = 0.5;
        public static final double CLIMBER_READY_FOR_NEXT_RUNG_HEIGHT = Units.inchesToMeters(5.0);
        public static final double CLIMBER_MAXIMUM_TRAVEL = 0.619;
        public static final double CLIMBER_SETPOINT_TOLERANCE = 0.01;
        public static final double MAX_CLIMBER_TRAVEL_SPEED = 6380.0 * CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER;

        // Other
        public static final TalonFXInvertType TOP_CLIMBER_MOTOR_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType BOTTOM_CLIMBER_MOTOR_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
    }

    // Constants pertaining to the vision subsystem go here
    public static class Vision {

        // Strings
        public static final String CAMERA_NAME = "BreadCam";

        // Measurements/Constants
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(31.0);
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104.0);
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30.0);
    }

    // Constants pertaining to the autonomus period of the match
    public static class Autonomus {

        // Circular Sweep Setpoints
        public static final double CIRCULAR_SWEEP_FLYWHEEL_VELOCITY = 1725.0;
        public static final double CIRCULAR_SWEEP_HOOD_ANGLE = 23.0;

        public static final double RETURN_SHOT_FLYWHEEL_VELOCITY = 1725.0;
        public static final double RETURN_SHOT_HOOD_ANGLE = 23.0;

        // Two Cargo Auto Setpoints
        public static final double TWO_SHOT_FLYWHEL_VELOCITY = 1700.0;
        public static final double TWO_SHOT_HOOD_ANGLE = 24.0;

        // Three Cargo Auto Setpoints
        public static final double THREE_SHOT_FLYWHEEL_VELOCITY = 1725.0;
        public static final double THREE_SHOT_HOOD_ANGLE = 25.0;

        // Five Cargo Auto Setpoints
        public static final double FIRST_SHOT_FLYWHEEL_VELOCITY = 1450.0;
        public static final double FIRST_SHOT_HOOD_ANGLE = 24.0;

        public static final double SECOND_SHOT_FLYWHEEL_VELOCITY = 1725.0;
        public static final double SECOND_SHOT_HOOD_ANGLE = 24.0;

        // Measurements
        public static final Translation2d FIELD_TO_TARGET = new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13.5));
        public static final double BALL_FLIGHT_TIME = 1.25;
        public static final double RADIUS_TO_BACK_BUMPER = Units.inchesToMeters(137.5);
        public static final double RADIUS_TO_ROBOT_CENTER = RADIUS_TO_BACK_BUMPER - Drive.ROBOT_LENGTH/2;
        public static final double LIMELIGHT_MOUNTING_ANGLE_ERROR = 3.41; // Degrees
    }
}
