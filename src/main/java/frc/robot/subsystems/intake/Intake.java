package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends StateMachine {

    IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();
    String name;

    // State variables
    SystemState mSystemState = SystemState.IDLE_RETRACTED;
    boolean spit = false;
    
    enum SystemState {
        IDLE_RETRACTED, 
        IDLE_EXTENDED,
        SUCK_EXTENDED,
        SPIT_RETRACTED, 
        SPIT_EXTENDED
    }

    // Constructs a new intake object 
    public Intake(IntakeIO io, String name) { 
        this.io = io;
        this.name = name;
    }

    // Requests the intake to suck while extended
    public void requestIntake() {
        mSystemState = SystemState.SUCK_EXTENDED;
    }

    // Requests the intake to outtake while retracted
    public void requestOuttakeRetracted(boolean spit) {
        this.spit = spit;
        mSystemState = SystemState.SPIT_RETRACTED;
    }

    // Requests the intake to outtake while extended
    public void requestOuttakeExtended(boolean spit) {
        this.spit = spit;
        mSystemState = SystemState.SPIT_EXTENDED;
    }

    // Requests the intake to idle while retracted
    public void requestIdleRetracted() {
        mSystemState = SystemState.IDLE_RETRACTED;
    }

    // Requests the intake to idle while extended
    public void requestIdleExtended() {
        mSystemState = SystemState.IDLE_EXTENDED;
    }

    @Override
    protected void init() { }

    @Override
    protected void log() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs(name, inputs);
    }

    @Override
    protected void applyOutputs() {
        if (mSystemState == SystemState.IDLE_RETRACTED) {
            io.spin(0.0);
            io.extend(false);
        } else if (mSystemState == SystemState.IDLE_EXTENDED) {
            io.spin(0.0);
            io.extend(true);
        } else if (mSystemState == SystemState.SUCK_EXTENDED) {
            io.spin(1.0);
            io.extend(true);
        } else if (mSystemState == SystemState.SPIT_RETRACTED) {
            io.spin(spit ? -0.3 : -1.0);
            io.extend(false);
        } else if (mSystemState == SystemState.SPIT_EXTENDED) {
            io.spin(spit ? -0.3 : -1.0);
            io.extend(true);
        } 
    }

    @Override
    protected void handleTransitions() { }

}
