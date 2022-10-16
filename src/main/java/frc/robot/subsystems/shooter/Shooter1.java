package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

import static frc.robot.Constants.Flywheel.*;
import static frc.robot.Constants.Hood.*;

public class Shooter1 extends StateMachine {

    ShooterIO io;
    ShooterIOInputs inputs = new ShooterIOInputs();
    
    ShooterState mSystemState = ShooterState.HOMING;

    // General Variables
    private double flywheelCalibration = FLYWHEEL_CALIBRATION;

    // State Variables
    private double mStateStartTime = 0.0;
    private boolean requestHome = false;
    private boolean requestShoot = false;
    private double flywheelRPMSetpoint = 0.0;
    private double hoodDegSetpoint = 0.0;


    enum ShooterState {
        HOMING,
        IDLE, 
        APPROACHING_SETPOINT, 
        STABALIZING,
        AT_SETPOINT
    }

    public void requestHome() {
        requestHome = true;
    }

    public void requestShoot(double flywheelRPM, double hoodDeg) {
        if (flywheelRPM == 0.0) {
            requestIdle(); // Request idle because the flywheel rpm is 0.0
        } else {
            requestShoot = true;
            this.flywheelRPMSetpoint = flywheelRPM * flywheelCalibration;
            this.hoodDegSetpoint = hoodDeg + 0.5;
        }
    }

    public void requestIdle() {
        requestShoot = false;
    }

    @Override
    protected void init() {
        mStateStartTime = BreadUtil.getFPGATimeSeconds();
    }

    @Override
    protected void log() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Shooter", inputs);
        flywheelCalibration = SmartDashboard.getNumber("Flywheel Calibration", FLYWHEEL_CALIBRATION);
    }

    @Override
    protected void applyOutputs() {
        if (mSystemState == ShooterState.HOMING) {
            io.setHoodVoltage(-1.5);
            io.setFlywheelVel(0.0);
        } else if (mSystemState == ShooterState.IDLE) {
            io.setHoodPos(HOOD_IDLE_POS);
            io.setFlywheelVel(SHOOTER_IDLE_VEL);
        } else if (mSystemState == ShooterState.APPROACHING_SETPOINT) {
            io.setHoodPos(hoodDegSetpoint);
            io.setFlywheelVel(flywheelRPMSetpoint);
        } else if (mSystemState == ShooterState.STABALIZING) {
            io.setHoodPos(hoodDegSetpoint);
            io.setFlywheelVel(flywheelRPMSetpoint);
        } else if (mSystemState == ShooterState.AT_SETPOINT) {
            io.setHoodPos(hoodDegSetpoint);
            io.setFlywheelVel(flywheelRPMSetpoint);
        }
    }

    @Override
    protected void handleTransitions() {
        ShooterState nextState = mSystemState;

        if (mSystemState == ShooterState.HOMING) {
            if (hoodLimitTriggered()) {
                exitHomingSequence();
                nextState = ShooterState.IDLE;
            }
        } else if (mSystemState == ShooterState.IDLE) {
            if (requestHome) {
                beginHomingSequence();
                nextState = ShooterState.HOMING;
            } else if (requestShoot) {
                nextState = ShooterState.APPROACHING_SETPOINT;
            } 
        } else if (mSystemState == ShooterState.APPROACHING_SETPOINT) {
            if (requestHome) {
                beginHomingSequence();
                nextState = ShooterState.HOMING;
            } else if (!requestShoot) {
                nextState = ShooterState.IDLE;
            } else if (atFlywheelSetpoint()&&atHoodSetpoint()) {
                nextState = ShooterState.STABALIZING;
            }
        } else if (mSystemState == ShooterState.STABALIZING) {
            if (requestHome) {
                beginHomingSequence();
                nextState = ShooterState.HOMING;
            } else if (!requestShoot) {
                nextState = ShooterState.IDLE;
            } else if (BreadUtil.getFPGATimeSeconds() - mStateStartTime >= 0.25) {
                nextState = ShooterState.AT_SETPOINT;
            }
        } else if (mSystemState == ShooterState.AT_SETPOINT) {
            if (requestHome) {
                beginHomingSequence();
                nextState = ShooterState.HOMING;
            } else if (!requestShoot) {
                nextState = ShooterState.IDLE;
            } else if (!atFlywheelSetpoint()||!atHoodSetpoint()) {
                nextState = ShooterState.APPROACHING_SETPOINT;
            }
        }

        if (nextState != mSystemState) {
            mSystemState = nextState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }
    }

    /* Helper methods for trasitioning in and out of states */
    private void beginHomingSequence() {
        io.setHoodCurrentLimits(5, 10.0);
    }   

    private void exitHomingSequence() {
        io.setHoodCurrentLimits(30, 40.0);
        io.resetHood(0.0);
        io.setHoodVoltage(0.0);
        requestHome = false;
    }


    /* User-facing getter methods */
    public boolean hoodLimitTriggered() {
        return inputs.hoodLimitTriggered;
    }

    public double getFlywheelVelocity() {
        return inputs.flywheelVelocityRotationsPerSecond;
    }

    public double getFlywheelSetpoint() {
        return flywheelRPMSetpoint;
    }

    public double getHoodPosition() {
        return inputs.hoodPosDeg;
    }

    public double getHoodSetpoint() {
        return hoodDegSetpoint;
    }

    public boolean atFlywheelSetpoint() {
        return BreadUtil.atReference(getFlywheelVelocity(), getFlywheelSetpoint(), 50.0, true);
    }

    public boolean atHoodSetpoint() {
        return BreadUtil.atReference(getHoodPosition(), getHoodSetpoint(), 0.5, true);
    }

    
}
