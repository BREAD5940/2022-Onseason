package frc.robot.autonomus;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;

public class Trajectories {

    public static Trajectory allianceSideSemiCircle = generateTrajectory(
        true, 
        List.of(
            new Pose2d(Units.feetToMeters(21.838), Units.feetToMeters(19.19), Rotation2d.fromDegrees(135)),
            new Pose2d(Units.feetToMeters(17.279), Units.feetToMeters(21.788), Rotation2d.fromDegrees(-111.473)),
            new Pose2d(Units.feetToMeters(15.423), Units.feetToMeters(10.819), Rotation2d.fromDegrees(-83.105)),
            new Pose2d(Units.feetToMeters(19.636), Units.feetToMeters(4.267), Rotation2d.fromDegrees(-48.244)),
            new Pose2d(Units.feetToMeters(24.385), Units.feetToMeters(2.022), Rotation2d.fromDegrees(-5.029))
        ), 1.0, 0.5, 0.0, 0.0
    );

    public static Trajectory advanceToHumanPlayerStation = generateTrajectory(
        true, 
        List.of(
            new Pose2d(Units.feetToMeters(25.063), Units.feetToMeters(1.498), Rotation2d.fromDegrees(134.196)),
            new Pose2d(Units.feetToMeters(5.355), Units.feetToMeters(4.39), Rotation2d.fromDegrees(-135))
        ), 1.0, 0.5, 0.0, 0.0
    );

    public static Trajectory returnFromHumanPlayerStation = generateTrajectory(
        true, 
        List.of(
            new Pose2d(Units.feetToMeters(5.355), Units.feetToMeters(4.39), Rotation2d.fromDegrees(20)),
            new Pose2d(Units.feetToMeters(17.683), Units.feetToMeters(8.899), Rotation2d.fromDegrees(20))
        ), 1.0, 0.5, 0.0, 0.0
    );

    public static Trajectory twoCargoLeftTarmac = generateTrajectory(
        true, 
        List.of(
            new Pose2d(5.877, 4.854, new Rotation2d(-3.114)),
            new Pose2d(5.006, 5.14, new Rotation2d(1.126)),
            new Pose2d(5.22, 6.081, new Rotation2d(1.052))
        ), 2.0, 1.0, 0.0, 0.0
    );

    public static Trajectory getFirstOpposingCargoBilliards = generateTrajectory(
        true, 
        List.of(
            new Pose2d(5.22, 6.081, new Rotation2d(1.116)),
            new Pose2d(6.224, 7.022, new Rotation2d(0.384))
        ), 3.0, 2.0, 0.0, 0.0
    );

    public static Trajectory prepareToUnstageCargoBilliards = generateTrajectory(
        true,
        List.of(
            new Pose2d(6.224, 7.022, new Rotation2d(-0.226)),
            new Pose2d(7.398 - Units.inchesToMeters(4.0), 6.8, new Rotation2d(-0.137))
        ), 2.8, 1.8, 0.0, 0.0
    );

    public static Trajectory getUnstagedBallBilliards = generateTrajectory(
        true, 
        List.of(
            new Pose2d(7.398 - Units.inchesToMeters(4.0), 6.8, new Rotation2d(1.666)),
            new Pose2d(7.352 - Units.inchesToMeters(4.0), 7.324 - Units.inchesToMeters(3.0), new Rotation2d(1.649))
        ), 2.8, 1.8, 0.0, 0.0
    );

    public static Trajectory returnUnstagedBallBilliards = generateTrajectory(
        true, 
        List.of(
            new Pose2d(7.352 - Units.inchesToMeters(4.0), 7.324 - Units.inchesToMeters(3.0), new Rotation2d(-2.623)),
            new Pose2d(6.203, 6.858, new Rotation2d(-2.431))
        ), 2.8, 1.8, 0.0, 0.0
    );

    public static Trajectory getFirstOpposingCargoTwoCargoDefensive = generateTrajectory(
        true, 
        List.of(
            new Pose2d(5.071 + Units.inchesToMeters(8.0), 6.07, new Rotation2d(1.052)),
            new Pose2d(5.931 + Units.inchesToMeters(8.0), 6.88, new Rotation2d(0.248))
        ), 3.0, 2.0, 0.0, 0.0
    );

    public static Trajectory getSecondOpposingCargoTwoCargoDefensive = generateTrajectory(
        true, 
        List.of(
            new Pose2d(5.931 + Units.inchesToMeters(8.0), 6.88, new Rotation2d(-3.027)),
            new Pose2d(4.853, 4.975, new Rotation2d(-1.445)),
            new Pose2d(4.93, 3.708, new Rotation2d(-1.465))
        ), 3.0, 2.0, 0.0, 0.0
    );

    public static Trajectory spitOpposingCargoTwoCargoDefensive = generateTrajectory(
        true, 
        List.of(
            new Pose2d(4.93, 3.708, new Rotation2d(1.642)),
            new Pose2d(4.745, 6.823, new Rotation2d(1.666))
        ), 3.0, 2.0, 0.0, 0.0
    );

    public static Trajectory getFirstCargoRightTarmac = generateTrajectory(
        true, 
        List.of(
            new Pose2d(7.574, 1.796, new Rotation2d(-1.516)),
            new Pose2d(7.619, 0.815, new Rotation2d(-1.528))
        ), 2.5, 2.0, 0.0, 0.0
    );

    public static Trajectory returnFirstCargoRightTarmac = generateTrajectory(
        true, 
        List.of(
            new Pose2d(7.619, 0.815, new Rotation2d(1.626)),
            new Pose2d(7.641, 1.53, new Rotation2d(1.557))
        ), 2.5, 2.0, 0.0, 0.0
    );

    public static Trajectory getThirdBallRightTarmac = generateTrajectory(
        true, 
        List.of(
            new Pose2d(7.641, 1.53, new Rotation2d(-2.502)),
            new Pose2d(5.169, 1.974, new Rotation2d(2.573))
        ), 2.5, 2.0, 0.0, 0.0
    );

    public static Trajectory advanceToHumanPlayerStationAfterThreeBall = generateTrajectory(
        true, 
        List.of(
            new Pose2d(5.169, 1.974, new Rotation2d(-3.002)),
            new Pose2d(1.286+Units.inchesToMeters(8.0-6.0+6.0), 1.462+Units.inchesToMeters(8.0+6.0+6.0), new Rotation2d(-2.5))
        ), 3.5, 2.5, 0.0, 0.0
    );


    public static Trajectory returnFromHumanPlayerStationAfterThreeBall = generateTrajectory(
        true, 
        List.of(
            new Pose2d(1.286+Units.inchesToMeters(8.0-6.0+6.0), 1.462+Units.inchesToMeters(8.0+6.0+6.0), new Rotation2d(0.17)),
            new Pose2d(5.017, 2.1, new Rotation2d(0.222))
        ), 3.5, 2.5, 0.0, 0.0
    );

    public static Trajectory goBackFromLeftTarmac = generateTrajectory(
        true, 
        List.of(
            new Pose2d(6.754, 5.997, new Rotation2d(2.178)),
            new Pose2d(5.929, 7.37, new Rotation2d(2.091))
        ), 2.0, 1.0, 0.0, 0.0
    );

    public static Trajectory generateTrajectory(boolean clampedCubic, List<Pose2d> points, double maxVel, double maxAccel, double startVel, double endVel, TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        for (TrajectoryConstraint c : constraints) {
            config.addConstraint(c);
        }
        config.setStartVelocity(startVel);
        config.setEndVelocity(endVel);
        if (!clampedCubic) {
            return TrajectoryGenerator.generateTrajectory(points.stream().map(point -> new Pose2d(point.getTranslation(), point.getRotation())).collect(Collectors.toList()), config);
        } else {
            List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
            for (int i=1; i<points.size()-1; i++) {
                interiorPoints.add(points.get(i).getTranslation());
            }
            return TrajectoryGenerator.generateTrajectory(points.get(0), interiorPoints, points.get(points.size()-1), config);
        }
    }
    
}
