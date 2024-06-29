package frc.robot.subsystems.carrier;

import org.littletonrobotics.junction.AutoLog;

public interface CarrierIO {

    @AutoLog
    public static class CarrierIOInputs {
        double appliedVolts = 0.0;
        double velocity = 0.0;
        boolean isPiecePresent = false;
    }
    public void updateInputs(CarrierIOInputs inputs);

    public default void setVoltage(double voltage) {}

}
