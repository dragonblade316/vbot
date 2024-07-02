package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double appliedVolts;

        //putting this here in case I decide to try out closed loop control for the intake
        public double velocity;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    public default void setVoltage(double voltage) {}
    public default void setVelocity(double radiansPerSecond, double ffVoltage) {}
    public default void setPID(double kp, double ki, double kd) {}
}
