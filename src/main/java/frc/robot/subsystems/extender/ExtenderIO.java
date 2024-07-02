package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {
    @AutoLog
    public static class ExtenderIOInputs {
        public double appliedVoltage = 0;
        public double metersExtended = 0;
    }
    public void updateInputs(ExtenderIOInputs inputs);

    public default void setVoltage(double voltage) {}
    public default void zeroEncoder() {}
}
