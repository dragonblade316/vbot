package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
    DCMotorSim motor;
    double encoder = 0.0;
    double voltage = 0.0;

    public ClimberIOSim() {
        motor = new DCMotorSim(DCMotor.getNEO(1), (double) 1 /75, 0.0001);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.encoder_value = encoder;
        inputs.atLowerLimit = (encoder <= 0);
        inputs.appliedVoltage = voltage;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setInputVoltage(voltage);
        //TODO: figure out how to increment the encoder
    }

    @Override
    public void zeroEncoder() {
        encoder = 0;
    }
}
