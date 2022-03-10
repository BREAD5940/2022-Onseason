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
    private final MK4iSwerveModule fl = new MK4iSwerveModule(DRIVE_IDS[0], STEER_IDS[0], AZIMUTH_CHANNELS[0], AZIMUTH_OFFSETS[0], DRIVES_ARE_REVERSED[0], STEERS_ARE_REVERSED[0], AZIMUTHS_ARE_REVERSED[0]);
    private final MK4iSwerveModule fr = new MK4iSwerveModule(DRIVE_IDS[1], STEER_IDS[1], AZIMUTH_CHANNELS[1], AZIMUTH_OFFSETS[1], DRIVES_ARE_REVERSED[1], STEERS_ARE_REVERSED[1], AZIMUTHS_ARE_REVERSED[1]);
    private final MK4iSwerveModule bl = new MK4iSwerveModule(DRIVE_IDS[2], STEER_IDS[2], AZIMUTH_CHANNELS[2], AZIMUTH_OFFSETS[2], DRIVES_ARE_REVERSED[2], STEERS_ARE_REVERSED[2], AZIMUTHS_ARE_REVERSED[2]);
    private final MK4iSwerveModule br = new MK4iSwerveModule(DRIVE_IDS[3], STEER_IDS[3], AZIMUTH_CHANNELS[3], AZIMUTH_OFFSETS[3], DRIVES_ARE_REVERSED[3], STEERS_ARE_REVERSED[3], AZIMUTHS_ARE_REVERSED[3]);
    
    // Kinematics & Odometry
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    // Field2d
    private Pose2d pose = odometry.getPoseMeters();
    private final Field2d field = new Field2d();

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
        // if (Math.abs(xMetersPerSecond) < 0.1 && Math.abs(yMetersPerSecond) < 0.1 && Math.abs(thetaRadiansPerSecond) < 0.1) {
        //     double cross = new Rotation2d(ROBOT_LENGTH, ROBOT_WIDTH).getRadians();
        //     states[0] = new SwerveModuleState(0.0, new Rotation2d(cross));
        //     states[1] = new SwerveModuleState(0.0, new Rotation2d(-cross));
        //     states[2] = new SwerveModuleState(0.0, new Rotation2d(-cross));
        //     states[3] = new SwerveModuleState(0.0, new Rotation2d(cross));
        // }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ROBOT_MAX_SPEED);
        fl.setState(states[0]);
        fr.setState(states[1]);
        bl.setState(states[2]);
        br.setState(states[3]);
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

    public double getXVelocity() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        );
        return speeds.vxMetersPerSecond;
    }

    public double getYVelocity() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        );
        return speeds.vyMetersPerSecond;
    }

    public double getVelocity() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        );
        return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Swerve Rotation", fl.getAngle());
        updateOdometry();
    }
    
}