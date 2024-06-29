package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVoltage = 0;
        public double encoder_value = 0;
        public boolean atLowerLimit = false;
    }
    public void updateInputs(ClimberIOInputs inputs);

    public default void setVoltage(double voltage) {}
    public default void zeroEncoder() {}
}
