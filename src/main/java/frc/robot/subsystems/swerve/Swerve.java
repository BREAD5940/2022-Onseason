package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.vision.RobotPositionHistory;

import static frc.robot.Constants.Drive.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {

    public double defaultDriveSpeed = 3.0;

    // Gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Modules
    private final MK4iSwerveModule fl = new MK4iSwerveModule(DRIVE_IDS[0], STEER_IDS[0], AZIMUTH_CHANNELS[0], AZIMUTH_OFFSETS[0], DRIVE_INVERT_TYPES[0], STEERS_ARE_REVERSED[0], AZIMUTHS_ARE_REVERSED[0], "FL");
    private final MK4iSwerveModule fr = new MK4iSwerveModule(DRIVE_IDS[1], STEER_IDS[1], AZIMUTH_CHANNELS[1], AZIMUTH_OFFSETS[1], DRIVE_INVERT_TYPES[1], STEERS_ARE_REVERSED[1], AZIMUTHS_ARE_REVERSED[1], "FR");
    private final MK4iSwerveModule bl = new MK4iSwerveModule(DRIVE_IDS[2], STEER_IDS[2], AZIMUTH_CHANNELS[2], AZIMUTH_OFFSETS[2], DRIVE_INVERT_TYPES[2], STEERS_ARE_REVERSED[2], AZIMUTHS_ARE_REVERSED[2], "BL");
    private final MK4iSwerveModule br = new MK4iSwerveModule(DRIVE_IDS[3], STEER_IDS[3], AZIMUTH_CHANNELS[3], AZIMUTH_OFFSETS[3], DRIVE_INVERT_TYPES[3], STEERS_ARE_REVERSED[3], AZIMUTHS_ARE_REVERSED[3], "BR");
    
    // Kinematics & Odometry
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);
    private final SwerveDriveOdometry matchOdometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    // Field2d
    private Pose2d pose = matchOdometry.getPoseMeters();
    public final Field2d field = new Field2d();

    // State variables
    private boolean atVisionHeadingSetpoint = false;

    // Constructs a new swerve object
    public Swerve() {
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("Traj-X-Error", 0.0);
        SmartDashboard.putNumber("Traj-Y-Error", 0.0);
        SmartDashboard.putNumber("Traj-Theta-Error", 0.0);
    }

    // Resets all of the swerve modules to use the absolute readings
    public void resetAllToAbsolute() {
        fl.resetToAbsolute();
        fr.resetToAbsolute();
        bl.resetToAbsolute();
        br.resetToAbsolute();
    }

    // Sets the desired speeds of the swerve drive
    public void setSpeeds(double xMetersPerSecond, double yMetersPerSecond, double thetaRadiansPerSecond) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            xMetersPerSecond, 
            yMetersPerSecond, 
            thetaRadiansPerSecond, 
            pose.getRotation()
        ));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ROBOT_MAX_SPEED);
        fl.setState(states[0]);
        // SmartDashboard.putNumber("FL Desired Velocity (Before Optimization and Continous Output)", states[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("FL Desired Angle (Before Optimization and Continous Output)", states[0].angle.getDegrees());
        // SmartDashboard.putNumber("FL Desired Velocity (After Optimization and Continous Output)", fl.getReferenceVelocity());
        // SmartDashboard.putNumber("FL Desired Angle (After Optimization and Continous Output", fl.getReferenceAngle());
        // SmartDashboard.putNumber("FL Motor Output Percent", fl.getDriveOutputPercent());
        fr.setState(states[1]);
        bl.setState(states[2]);
        br.setState(states[3]);
    }

    // Sets the drive PID slots to use for more agressive/soft feedback control
    public void setDriveSlots(int slot) {
        fl.setDriveSlot(slot);
        fr.setDriveSlot(slot);
        bl.setDriveSlot(slot);
        br.setDriveSlot(slot);
    }

    // Returns the raw gyro angle; is negated to be counterclockwise positive
    public double getRawGyro() {
        return -gyro.getAngle();
    }

    // Overload to set a robot relative speed
    public void setSpeeds(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ROBOT_MAX_SPEED);
        fl.setState(states[0]);
        fr.setState(states[1]);
        bl.setState(states[2]);
        br.setState(states[3]);
    }

    // Resets match odometry
    public void reset(Pose2d newPose) {
        matchOdometry.resetPosition(newPose, gyro.getRotation2d());
        pose = matchOdometry.getPoseMeters();
    }
    
    // Updates match odometry
    public void updateOdometry() {
        pose = matchOdometry.update(
            gyro.getRotation2d(),
            fl.getState(), 
            fr.getState(), 
            bl.getState(),
            br.getState()
        );
        RobotPositionHistory.update(BreadUtil.getFPGATimeSeconds(), pose);
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }

    // Returns the match pose
    public Pose2d getPose() {
        return pose;
    }

    // Sets the neutral mode of the drive motors
    public void setNeutralModes(NeutralMode mode) {
        fl.drive.setNeutralMode(mode);
        fr.drive.setNeutralMode(mode);
        bl.drive.setNeutralMode(mode);
        br.drive.setNeutralMode(mode);
    }

    // Sets whether or not the drivetrain is at its vision reference
    public void setAtVisionHeadingSetpoint(boolean set) {
        atVisionHeadingSetpoint = set;
    }

    // Returns whether or not the drivetrain is at its vision reference
    public boolean getAtVisionHeadingSetpoint() {
        return atVisionHeadingSetpoint;
    }

    // Returns the ROBOT RELATIVE speed of the drivetrain
    public Translation2d getVelocity() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        );
        return new Translation2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond
        );
    }

    // Periodically updates odometry and posts values to smart dashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Get Raw Gyro Angle", getRawGyro());
        updateOdometry();
        SmartDashboard.putData(field);
    }
    
}