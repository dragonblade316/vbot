package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {
    @AutoLog
    public static class ExtenderIOInputs {
        public double appliedVoltage = 0;
        public double metersExtended = 0;
        public double velocityMetersPerSecond = 0;
    }
    public default void updateInputs(ExtenderIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
    public default void setPosition(double meters) {}
    public default void setPID(double kp, double ki, double kd) {}
    public default void zeroEncoder() {}
}
