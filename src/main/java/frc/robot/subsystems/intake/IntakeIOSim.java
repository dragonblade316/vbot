package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    DCMotorSim motor;

    public IntakeIOSim() {
        motor = new DCMotorSim(DCMotor.getNEO(1), 1/5, 0.001);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }
}
