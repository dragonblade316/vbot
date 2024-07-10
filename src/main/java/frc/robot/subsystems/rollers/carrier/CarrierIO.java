package frc.robot.subsystems.rollers.carrier;

import org.littletonrobotics.junction.AutoLog;

public interface CarrierIO {

    @AutoLog
    public static class CarrierIOInputs {
        double appliedVolts = 0.0;
        double velocityRPM = 0.0;
        double positionRotations = 0.0;
        boolean isPiecePresent = false;
    }
    public default void updateInputs(CarrierIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
    public default void setVelocity(double rpm, double ffVoltage) {}
    public default void setPID(double kp, double ki, double kd) {}

}