// package frc.robot.autonomus;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.autonomus.routines.SixCargoSweep;
// import frc.robot.autonomus.routines.TwoCargoTurnToRight;
// import frc.robot.statemachines.Superstructure;
// import frc.robot.subsystems.intake.IntakePneumatics;
// import frc.robot.subsystems.swerve.Swerve;

// public class AutonomusSelector {

//     private SendableChooser<SequentialCommandGroup> autonomusSelector = new SendableChooser<SequentialCommandGroup>();

//     public AutonomusSelector(Superstructure superstructure, IntakePneumatics intakePneumatics, Swerve swerve) {
//         autonomusSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
//         autonomusSelector.addOption("SIX_CARGO_SWEEP", new SixCargoSweep(superstructure, intakePneumatics, swerve));
//         autonomusSelector.addOption("TWO_CARGO_TURN_TO_RIGHT", new TwoCargoTurnToRight(superstructure, intakePneumatics, swerve));
//     }

//     public SequentialCommandGroup get() {
//         return autonomusSelector.getSelected();
//     }
    
// }
