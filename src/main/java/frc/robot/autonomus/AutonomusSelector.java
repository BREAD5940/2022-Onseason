package frc.robot.autonomus;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.routines.BilliardsThreeBallLeftTarmac;
import frc.robot.autonomus.routines.BumpyBilliardsThreeBallLeftTarmac;
import frc.robot.autonomus.routines.FiveCargoRightTarmac;
import frc.robot.autonomus.routines.OneCargoLeftTarmac;
import frc.robot.autonomus.routines.TwoCargoLeftTarmacDefensive;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomusSelector {

    private SendableChooser<SequentialCommandGroup> autonomusSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomusSelector(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        autonomusSelector.setDefaultOption(
            "DO_NOTHING", 
            new SequentialCommandGroup()
        );
        autonomusSelector.addOption(
            "TWO_CARGO_DEFENSIVE_LEFT_TARMAC", 
            new TwoCargoLeftTarmacDefensive(swerve, shooter, leftIntake, rightIntake, gutNeck)
        );
        autonomusSelector.addOption(
            "FIVE_CARGO_RIGHT_TARMAC", 
            new FiveCargoRightTarmac(swerve, shooter, leftIntake, rightIntake, gutNeck)
        );
        autonomusSelector.addOption(
            "SMOOTH_BILLIARD_THREE_BALL_LEFT_TARMAC", 
            new BilliardsThreeBallLeftTarmac(swerve, shooter, leftIntake, rightIntake, gutNeck)
        );
        autonomusSelector.addOption(
            "BUMPY_BILLIARD_THREE_BALL_LEFT_TARMAC", 
            new BumpyBilliardsThreeBallLeftTarmac(swerve, shooter, leftIntake, rightIntake, gutNeck)
        );
        autonomusSelector.addOption(
            "ONE_CARGO_LEFT_TARMAC", 
            new OneCargoLeftTarmac(swerve, shooter, leftIntake, rightIntake, gutNeck)
        );
        SmartDashboard.putData("Autonomus Selector", autonomusSelector);
    }

    public SequentialCommandGroup get() {
        return autonomusSelector.getSelected();
    }
    
}
