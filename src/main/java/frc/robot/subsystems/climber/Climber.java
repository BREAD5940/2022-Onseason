package frc.robot.subsystems.climber;

import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends StateMachine {

    private final ClimberIO io;
    private ClimberIOInputs inputs = new ClimberIOInputs();

    private SystemState mSystemState = SystemState.STARTING_CONFIGURATION;

    // State variables
    private boolean mRequestNextState = false;
    private boolean mRequestPrevState = false;
    public double mStateStartTime = 0.0;

    enum SystemState {
        STARTING_CONFIGURATION,
        READY_FOR_MID,
        CLIMB_TO_MID,
        HOLD_ON_MID,
        RELEASE_MID_SMALL, 
        RELEASE_MID,
        TILT_TOWARDS_HIGH, 
        EXTEND_TOWARDS_HIGH, 
        READY_FOR_HIGH,
        POP_TO_HIGH,
        CLIMB_TO_HIGH,
        HOLD_ON_HIGH,
        RELEASE_HIGH_SMALL, 
        RELEASE_HIGH, 
        EXTEND_TOWARDS_TRAVERSAL,
        READY_FOR_TRAVERSAL, 
        POP_TO_TRAVERSAL
    }

    // Constructs a new climber object
    public Climber(ClimberIO io) {
        this.io = io;
    }

    // Requests the climber to move to its next state
    public void requestNextState() {
        mRequestNextState = true;
    }

    public void requestPrevState() {
        mRequestPrevState = true;
    }


    @Override
    protected void init() {
        mStateStartTime = BreadUtil.getFPGATimeSeconds();
        io.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    protected void log() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Climber", inputs);
        // SmartDashboard.putString("Climber State", mSystemState.name());
        // SmartDashboard.putNumber("Climber Height", getHeight());
    }

    @Override
    protected void applyOutputs() {
        if (mSystemState == SystemState.STARTING_CONFIGURATION) {
            io.setHeight(CLIMBER_RETRACTED_HEIGHT, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.READY_FOR_MID) {
            io.setHeight(CLIMBER_MID_RUNG_HEIGHT, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.CLIMB_TO_MID) {
            io.setHeight(CLIMBER_RETRACTED_HEIGHT, true);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.HOLD_ON_MID) {
            io.setHeight(CLIMBER_RETRACTED_HEIGHT, true);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.RELEASE_MID_SMALL) {
            io.setHeight(CLIMBER_RELEASE_SMALL, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.RELEASE_MID) {
            io.setHeight(CILMBER_RELEASE_FROM_RUNG, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.TILT_TOWARDS_HIGH) {
            io.setHeight(CLIMBER_TILT_TOWARDS_HIGH, false);
            io.setPistonsForward(false);
        } else if (mSystemState == SystemState.EXTEND_TOWARDS_HIGH) {
            io.setHeight(CLIMBER_EXTEND, false);
            io.setPistonsForward(false);
        } else if (mSystemState == SystemState.READY_FOR_HIGH) {
            io.setHeight(CLIMBER_EXTEND, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.POP_TO_HIGH) {
            io.setHeight(CLIMBER_EXTEND - Units.inchesToMeters(12.0), true);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.CLIMB_TO_HIGH) {
            io.setHeight(CLIMBER_RETRACTED_HEIGHT, true);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.RELEASE_HIGH_SMALL) {
            io.setHeight(CLIMBER_RELEASE_SMALL, false);
            io.setPistonsForward(true);
        } if (mSystemState == SystemState.RELEASE_HIGH) {
            io.setHeight(CILMBER_RELEASE_FROM_RUNG, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.EXTEND_TOWARDS_TRAVERSAL) {
            io.setHeight(CLIMBER_EXTEND, false);
            io.setPistonsForward(true);
        } else if (mSystemState == SystemState.READY_FOR_TRAVERSAL) {
            io.setHeight(CLIMBER_EXTEND, false);
            io.setPistonsForward(false);
        } else if (mSystemState == SystemState.POP_TO_TRAVERSAL) {
            io.setHeight(CLIMBER_EXTEND - Units.inchesToMeters(12.0), true);
            io.setPistonsForward(false);
        }
    }

    @Override
    protected void handleTransitions() {
        SystemState nextState = mSystemState;

        if (mSystemState == SystemState.STARTING_CONFIGURATION) {
            if (mRequestNextState) {
                nextState = SystemState.READY_FOR_MID;
            }
        } else if (mSystemState == SystemState.READY_FOR_MID) {
            if (mRequestNextState) {
                nextState = SystemState.CLIMB_TO_MID;
            } 
            if (mRequestPrevState) {
                nextState = SystemState.STARTING_CONFIGURATION;
            }
        } else if (mSystemState == SystemState.CLIMB_TO_MID) {
            if (BreadUtil.atReference(getHeight(), CLIMBER_RETRACTED_HEIGHT, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextState = SystemState.HOLD_ON_MID;
            }
        } else if (mSystemState == SystemState.HOLD_ON_MID) {
            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.5) {
                 nextState = SystemState.RELEASE_MID_SMALL;
            }
        } else if (mSystemState == SystemState.RELEASE_MID_SMALL) {
            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.5) {
                nextState = SystemState.RELEASE_MID;
           }
        } else if (mSystemState == SystemState.RELEASE_MID) {
            if (mRequestNextState) {
                nextState = SystemState.TILT_TOWARDS_HIGH;
            }
            if (mRequestPrevState) {
                nextState = SystemState.STARTING_CONFIGURATION;
            }
        } else if (mSystemState == SystemState.TILT_TOWARDS_HIGH) {
            if (mRequestNextState) {
                nextState = SystemState.EXTEND_TOWARDS_HIGH;
            }
            if (mRequestPrevState) {
                nextState = SystemState.RELEASE_MID;
            }
        } else if (mSystemState == SystemState.EXTEND_TOWARDS_HIGH) {
            if (mRequestNextState) {
                nextState = SystemState.READY_FOR_HIGH;
            }
            if (mRequestPrevState) {
                nextState = SystemState.TILT_TOWARDS_HIGH;
            }
        } else if (mSystemState == SystemState.READY_FOR_HIGH) {
            if (mRequestNextState) {
                nextState = SystemState.POP_TO_HIGH;
            }
            if (mRequestPrevState) {
                nextState = SystemState.EXTEND_TOWARDS_HIGH;
            }
        } else if (mSystemState == SystemState.POP_TO_HIGH) {
            if (mRequestNextState) {
                nextState = SystemState.CLIMB_TO_HIGH;
            }
            if (mRequestPrevState) {
                nextState = SystemState.READY_FOR_HIGH;
            }
        } else if (mSystemState == SystemState.CLIMB_TO_HIGH) {
            if (BreadUtil.atReference(getHeight(), CLIMBER_RETRACTED_HEIGHT, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextState = SystemState.HOLD_ON_HIGH;
            }
        } else if (mSystemState == SystemState.HOLD_ON_HIGH) {
            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.5) {
                nextState = SystemState.RELEASE_HIGH_SMALL;
            }
        } else if (mSystemState == SystemState.RELEASE_HIGH_SMALL) {
            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.5) {
                nextState = SystemState.RELEASE_HIGH;
            }
        } else if (mSystemState == SystemState.RELEASE_HIGH) {
            if (mRequestNextState) {
                nextState = SystemState.EXTEND_TOWARDS_TRAVERSAL;
            }
            if (mRequestPrevState) {
                nextState = SystemState.POP_TO_HIGH;
            }
        } else if (mSystemState == SystemState.EXTEND_TOWARDS_TRAVERSAL) {
            if (mRequestNextState) {
                nextState = SystemState.READY_FOR_TRAVERSAL;
            }
            if (mRequestPrevState) {
                nextState = SystemState.RELEASE_HIGH;
            }
        } else if (mSystemState == SystemState.READY_FOR_TRAVERSAL) {
            if (mRequestNextState) {
                nextState = SystemState.POP_TO_TRAVERSAL;
            }
            if (mRequestPrevState) {
                nextState = SystemState.EXTEND_TOWARDS_TRAVERSAL;
            }
        } else if (mSystemState == SystemState.POP_TO_TRAVERSAL) {
            if (mRequestPrevState) {
                nextState = SystemState.READY_FOR_TRAVERSAL;
            }
        }

        mRequestNextState = false;
        mRequestPrevState = false;
        if (nextState != mSystemState) {
            mSystemState = nextState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }
    }

    /* User-facing getter methods */

    public double getHeight() {
        return inputs.posMeters;
    }

    // TODO TEMPORARY
    public void setEasy(boolean set) {
        io.setNeutralMode(set ? NeutralMode.Coast : NeutralMode.Brake);
    }
    
}
