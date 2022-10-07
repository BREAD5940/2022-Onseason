package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/* intake subsystem hardware interface */
public interface IntakeIO {
    /* contains all of the input data recieved from the hardware */
    public static class IntakeIOInputs implements LoggableInputs {
        public boolean solenoidsForward = false;
        public double appliedVoltage = 0.0; 
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;

        public void toLog(LogTable table) {
            table.put("SolenoidsForward", solenoidsForward);
            table.put("AppliedVoltage", appliedVoltage);
            table.put("CurrentAmps", currentAmps);
            table.put("TempCelcius", tempCelcius);
        } 

        public void fromLog(LogTable table) {
            solenoidsForward = table.getBoolean("SolenoidsForward", solenoidsForward);
            appliedVoltage = table.getDouble("AppliedVoltage", appliedVoltage);
            currentAmps = table.getDouble("CurrentAmps", currentAmps);
            tempCelcius = table.getDouble("TempCelcius", tempCelcius);
        }
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /* Spins the intake at the given percent */
    public default void spin(double percent) {}

    /* Sets the pistons forwards or reversed */
    public default void extend(boolean wantsIntakeExtended) {}


}
