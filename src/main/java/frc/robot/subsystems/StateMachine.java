package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// The StateMachine class is to be extended by any classes that would otherwise be subsystems in standard wpilib command-based projects. 
// It provides methods that make it easier to write statemachine-based code while staying somewhat true to the wpilib command-based paradigm.
public abstract class StateMachine extends SubsystemBase {

    // Variable to track whether the StateMachine has been initialized
    private boolean mInitialized = false;

    // The init() method is to be overriden by any subclasss
    // The init() method runs once
    // It's for initializing motors and variables that would otherwise not work in the constructor.
    protected abstract void init();

    // The log() method is to be overriden by any subclasses.
    // the log() method runs every loop cycle.
    // It's for updating inputs and logging anything else neccesary.
    protected abstract void log();

    // The applyOutputs() method is to be overriden by any subclasses. 
    // This method applies outputs based on state variables.
    // It is periodically called every loop cycle. 
    protected abstract void applyOutputs();
    
    // The handleTransitions() method is to be overriden by any subclasses.
    // This method applies outputs based on state variables.
    // It is periodically called every loop cycle.
    protected abstract void handleTransitions();

    // Periodically calls init() once and onLoop() and log() every loop cycle. 
    @Override
    public void periodic() {
        if (!mInitialized) {
            init();
            mInitialized = true;
        }
        log();
        applyOutputs();
        handleTransitions();
    }

}
