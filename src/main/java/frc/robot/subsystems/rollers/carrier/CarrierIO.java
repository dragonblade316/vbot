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

    //TODO: this will not acctually effect much on the inside but by putting this here we can run a speed intake on the real IO and pick up game pieces in sim 
    public default void setState(Carrier.CarrierGoal goal) {}
}
