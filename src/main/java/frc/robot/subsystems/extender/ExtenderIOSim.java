package frc.robot.subsystems.extender;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ExtenderIOSim implements ExtenderIO {
    ElevatorSim motor = new ElevatorSim(
            //TODO: get mesurements
            DCMotor.getNEO(1),
            1,
            0.5,
            0.0254,
            0.0,
            1,
            false,
            0.0);

    PIDController pid = new PIDController(0, 0, 0);
    boolean closedLoopControl = true;
    double voltage = 0;

    public ExtenderIOSim() {
        pid.setTolerance(0.1);
    }
    
    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        motor.update(0.02);

        inputs.metersExtended = motor.getPositionMeters();
        inputs.velocityMetersPerSecond = motor.getVelocityMetersPerSecond();
        inputs.appliedVoltage = voltage;

        if (closedLoopControl) {
            voltage = pid.calculate(motor.getPositionMeters());
            motor.setInputVoltage(voltage);
        }
    }

    public void setPosition(double meters) {
        closedLoopControl = true;
        pid.setSetpoint(meters);
    }

    public void setVoltage(double voltage) {
        closedLoopControl = false;
        motor.setInputVoltage(voltage);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);
    }










}
