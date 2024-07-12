package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
    ElevatorSim motor;

    PIDController pid = new PIDController(0, 0, 0);
    boolean closedLoopControl = true;
    double voltage = 0.0;


    public ClimberIOSim() {
        // motor = new DCMotorSim(DCMotor.getNEO(1), (double) 75, 0.0001);
        //TODO get Mesurements
        motor = new ElevatorSim(
            DCMotor.getNEO(1),
            75,
            0.5,
            0.0254,
            0.0,
            1,
            false,
            0.0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        motor.update(0.02);

        inputs.heightFromBaseMeters = motor.getPositionMeters();
        inputs.atLowerLimit = motor.getPositionMeters() <= 0;
        inputs.velocityMetersPerSecond = motor.getVelocityMetersPerSecond();
        inputs.appliedVoltage = voltage;

        if (closedLoopControl) {
            voltage = pid.calculate(motor.getPositionMeters());
            motor.setInputVoltage(voltage);
        }
    }

    @Override
    public void setPosition(double metersFromBase) {
        pid.setSetpoint(metersFromBase);
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setInputVoltage(voltage);
        //TODO: figure out how to increment the encoder
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);
    }

    @Override
    public void zeroEncoder() {
        motor.setState(0, 0);
    }
}
