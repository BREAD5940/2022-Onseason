package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Drive.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {

    // Gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Modules
    private final MK4iSwerveModule fl = new MK4iSwerveModule(DRIVE_IDS[0], STEER_IDS[0], AZIMUTH_CHANNELS[0], AZIMUTH_OFFSETS[0], DRIVE_INVERT_TYPES[0], STEERS_ARE_REVERSED[0], AZIMUTHS_ARE_REVERSED[0], "FL");
    private final MK4iSwerveModule fr = new MK4iSwerveModule(DRIVE_IDS[1], STEER_IDS[1], AZIMUTH_CHANNELS[1], AZIMUTH_OFFSETS[1], DRIVE_INVERT_TYPES[1], STEERS_ARE_REVERSED[1], AZIMUTHS_ARE_REVERSED[1], "FR");
    private final MK4iSwerveModule bl = new MK4iSwerveModule(DRIVE_IDS[2], STEER_IDS[2], AZIMUTH_CHANNELS[2], AZIMUTH_OFFSETS[2], DRIVE_INVERT_TYPES[2], STEERS_ARE_REVERSED[2], AZIMUTHS_ARE_REVERSED[2], "BL");
    private final MK4iSwerveModule br = new MK4iSwerveModule(DRIVE_IDS[3], STEER_IDS[3], AZIMUTH_CHANNELS[3], AZIMUTH_OFFSETS[3], DRIVE_INVERT_TYPES[3], STEERS_ARE_REVERSED[3], AZIMUTHS_ARE_REVERSED[3], "BR");
    
    // Kinematics & Odometry
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    // Field2d
    private Pose2d pose = odometry.getPoseMeters();
    private final Field2d field = new Field2d();

    // State variables
    private boolean atVisionHeadingSetpoint = false;

    public Swerve() {
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }

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

    public void setDriveSlots(int slot) {
        fl.setDriveSlot(slot);
        fr.setDriveSlot(slot);
        bl.setDriveSlot(slot);
        br.setDriveSlot(slot);
    }

    public double getRawGyro() {
        return -gyro.getAngle();
    }

    public void setSpeeds(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ROBOT_MAX_SPEED);
        fl.setState(states[0]);
        fr.setState(states[1]);
        bl.setState(states[2]);
        br.setState(states[3]);
    }

    public void reset(Pose2d newPose) {
        odometry.resetPosition(newPose, gyro.getRotation2d());
        pose = odometry.getPoseMeters();
    }

    public void updateOdometry() {
        pose = odometry.update(
            gyro.getRotation2d(),
            fl.getState(), 
            fr.getState(), 
            bl.getState(),
            br.getState()
        );
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setNeutralModes(NeutralMode mode) {
        fl.drive.setNeutralMode(mode);
        fr.drive.setNeutralMode(mode);
        bl.drive.setNeutralMode(mode);
        br.drive.setNeutralMode(mode);
    }

    public void setAtVisionHeadingSetpoint(boolean set) {
        atVisionHeadingSetpoint = set;
    }

    public boolean getAtVisionHeadingSetpoint() {
        return atVisionHeadingSetpoint;
    }

    public double[] getVelocities() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        );
        return new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond};
    }

    public double getXVelocity() {
        return getVelocities()[0];
    }

    public double getYVelocity() {
        return getVelocities()[1];
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Get Raw Gyro Angle", getRawGyro());
        updateOdometry();
    }
    
}