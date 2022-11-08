package frc.robot.subsystems.gutneck;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.sensors.ColorSensor.BallColor;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.gutneck.GutNeckIO.GutNeckIOInputs;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.Robot;

import static frc.robot.Constants.Gut.*;
import static frc.robot.Constants.Neck.*;

public class GutNeck1 extends StateMachine {

    GutNeckIO io;
    GutNeckIOInputs inputs = new GutNeckIOInputs();

    GutNeckStates mSystemState = GutNeckStates.IDLE_NO_CARGO;

    // State variables
    private double mStateStartTime = 0.0;
    private boolean acceptOpposingCargo = false;
    private boolean requestIntakeLeft = false;
    private boolean requestIntakeRight = false;
    private boolean requestShoot = false;
    private boolean requestSpitRight = false;
    private boolean requestSpitLeft = false;
    private boolean ballsExpelledFromNeck = false;

    private Timer shootingTimer = new Timer();
    private Timer stowingTimer = new Timer();

    enum GutNeckStates {
        IDLE_NO_CARGO,
        IDLE_ONE_CARGO,
        IDLE_TWO_CARGO,
        INTAKE_LEFT_NO_CARGO,
        INTAKE_RIGHT_NO_CARGO,
        INTAKE_LEFT_ONE_CARGO,
        INTAKE_RIGHT_ONE_CARGO, 
        STOW_ONE_CARGO_IN_NECK,
        SHOOT_CARGO,
        SPIT_LEFT, 
        SPIT_RIGHT
    }

    public void acceptOpposingCargo(boolean set) {
        acceptOpposingCargo = set;
    }

    public void requestIntakeLeft(boolean set) {
        requestIntakeLeft = set;
    }

    public void requestIntakeRight(boolean set) {
        requestIntakeRight = set;
    }
    
    public void requestShoot(boolean set) {
        requestShoot = set;
    }

    public void requestSpitLeft(boolean set) {
        requestSpitLeft = set;
    }

    public void requestSpitRight(boolean set) {
        requestSpitRight = set;
    }

    public void requestReset() {
        mSystemState = GutNeckStates.IDLE_NO_CARGO;
    }

    @Override
    protected void init() {
        mStateStartTime = BreadUtil.getFPGATimeSeconds();
    }

    @Override
    protected void log() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Shooter", inputs);
    }

    @Override
    protected void applyOutputs() { 
        if (mSystemState == GutNeckStates.IDLE_NO_CARGO) {
            io.setGutSpeedMetersPerSecond(0.0);
            io.setNeckSpeedMetersPerSecond(0.0); 
        } else if (mSystemState == GutNeckStates.IDLE_ONE_CARGO) {
            io.setGutSpeedMetersPerSecond(0.0);
            io.setNeckSpeedMetersPerSecond(0.0);
        } else if (mSystemState == GutNeckStates.IDLE_TWO_CARGO) {
            io.setGutSpeedMetersPerSecond(0.0);
            io.setNeckSpeedMetersPerSecond(0.0);
        } else if (mSystemState == GutNeckStates.INTAKE_LEFT_NO_CARGO) {
            io.setGutSpeedMetersPerSecond(GUT_INTAKING_SPEED);
            io.setNeckSpeedMetersPerSecond(0.0);
        } else if (mSystemState == GutNeckStates.INTAKE_RIGHT_NO_CARGO) {
            io.setGutSpeedMetersPerSecond(-GUT_INTAKING_SPEED);
            io.setNeckSpeedMetersPerSecond(0.0);
        } else if (mSystemState == GutNeckStates.INTAKE_LEFT_ONE_CARGO) {
            io.setGutSpeedMetersPerSecond(GUT_INTAKING_SPEED);
            io.setNeckSpeedMetersPerSecond(0.0);
        } else if (mSystemState == GutNeckStates.INTAKE_RIGHT_ONE_CARGO) {
            io.setGutSpeedMetersPerSecond(-GUT_INTAKING_SPEED);
            io.setNeckSpeedMetersPerSecond(0.0);
        } else if (mSystemState == GutNeckStates.STOW_ONE_CARGO_IN_NECK) {
            io.setGutSpeedMetersPerSecond(0.0); 
            io.setNeckSpeedMetersPerSecond(3.0);
        } else if (mSystemState == GutNeckStates.SHOOT_CARGO) {
            io.setGutSpeedMetersPerSecond(0.0);
            if (RobotContainer.shooter.getSystemState()==ShooterState.AT_SETPOINT) {
                io.setNeckSpeedMetersPerSecond(2.5);
            }  else {
                io.setNeckSpeedMetersPerSecond(0.0);
                ballsExpelledFromNeck = false;
            }

            if (!getBottomBeamBreakTriggered() && !getTopBeamBreakTriggered() && !ballsExpelledFromNeck) {
                ballsExpelledFromNeck = true;
                shootingTimer.start(); 
            }

            if (getBottomBeamBreakTriggered() || getTopBeamBreakTriggered()) {
                ballsExpelledFromNeck = false;
                shootingTimer.stop();
                shootingTimer.reset();
            }
        } else if (mSystemState == GutNeckStates.SPIT_LEFT) {
            io.setGutSpeedMetersPerSecond(-2.0);
            io.setNeckSpeedMetersPerSecond(-1.0);
        } else if (mSystemState == GutNeckStates.SPIT_RIGHT) {
            io.setGutSpeedMetersPerSecond(2.0);
            io.setNeckSpeedMetersPerSecond(-1.0);
        }
    }

    @Override
    protected void handleTransitions() {
        GutNeckStates nextSystemState = mSystemState;

        if (mSystemState == GutNeckStates.IDLE_NO_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestIntakeLeft) {
                nextSystemState = GutNeckStates.INTAKE_LEFT_NO_CARGO;
            } else if (requestIntakeRight) {
                nextSystemState = GutNeckStates.INTAKE_RIGHT_NO_CARGO;
            } 
        }  else if (mSystemState == GutNeckStates.IDLE_ONE_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot && RobotContainer.shooter.getSystemState() == ShooterState.AT_SETPOINT) {
                beginShootingSequence();
                nextSystemState = GutNeckStates.SHOOT_CARGO;
            } else if (requestIntakeLeft && !requestShoot) {
                nextSystemState = GutNeckStates.INTAKE_LEFT_ONE_CARGO;
            } else if (requestIntakeRight && !requestShoot) {
                nextSystemState = GutNeckStates.INTAKE_RIGHT_ONE_CARGO;
            }   
        } else if (mSystemState == GutNeckStates.IDLE_TWO_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot && RobotContainer.shooter.getSystemState() == ShooterState.AT_SETPOINT) {
                beginShootingSequence();
                nextSystemState = GutNeckStates.SHOOT_CARGO;
            } else if (!checkCargo(getMiddleColor())) { 
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            }
        } else if (mSystemState == GutNeckStates.INTAKE_LEFT_NO_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (!requestIntakeLeft) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            } else if (checkCargo(getMiddleColor())) {
                beginStowingSequence();
                nextSystemState = GutNeckStates.STOW_ONE_CARGO_IN_NECK;
            }
        } else if (mSystemState == GutNeckStates.INTAKE_RIGHT_NO_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (!requestIntakeRight) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            } else if (checkCargo(getMiddleColor())) {
                beginStowingSequence();
                nextSystemState = GutNeckStates.STOW_ONE_CARGO_IN_NECK;
            }
        } else if (mSystemState == GutNeckStates.INTAKE_LEFT_ONE_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (!requestIntakeLeft) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (checkCargo(getMiddleColor())) {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            }
        } else if (mSystemState == GutNeckStates.INTAKE_RIGHT_ONE_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (!requestIntakeRight) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (checkCargo(getMiddleColor())) {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            }
        } else if (mSystemState == GutNeckStates.STOW_ONE_CARGO_IN_NECK) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (getTopBeamBreakTriggered()) {
                exitStowingSequence();
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (stowingTimer.get() > 3.0) {
                exitStowingSequence();
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        } else if (mSystemState == GutNeckStates.SHOOT_CARGO) {
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (shootingTimer.get() > 0.5 && ballsExpelledFromNeck) {
                exitShootingSequence();
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        } else if (mSystemState == GutNeckStates.SPIT_LEFT) {
            if (!requestSpitLeft) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        } else if (mSystemState == GutNeckStates.SPIT_RIGHT) {
            if (!requestSpitRight) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        }

        if (nextSystemState != mSystemState) {
            mSystemState = nextSystemState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }
    }

    /* State Transition Helper Emails */
    private void beginShootingSequence() {
        shootingTimer.reset();
        ballsExpelledFromNeck = false;
    }

    private void exitShootingSequence() {
        shootingTimer.reset();
        shootingTimer.stop();
    }

    private void beginStowingSequence() {
        stowingTimer.reset();
        stowingTimer.start();
    }

    private void exitStowingSequence() {
        stowingTimer.reset();
        stowingTimer.stop();
    }

    private boolean checkCargo(BallColor color) { // TODO check what happens if you unplug the sensor
        if (acceptOpposingCargo) {
            return getBottomBeamBreakTriggered(); // Uses beam break
        } else {
            return color == Robot.allianceColor; // Uses color sensor
        }
    }

    /* Getters */
    public boolean getBottomBeamBreakTriggered() {
        return inputs.bottomBeamBreakBroken;
    }

    public boolean getTopBeamBreakTriggered() {
        return inputs.topBeamBreakBroken;
    }

    public BallColor getMiddleColor() {
        return inputs.detectedColor;
    }


    
}