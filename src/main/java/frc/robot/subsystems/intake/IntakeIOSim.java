package frc.robot.subsystems.intake;

import org.opencv.features2d.Feature2D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    DCMotorSim motor;
    PIDController feedback = new PIDController(0, 0, 0);


    private double voltage = 0;
    private double ffVoltage = 0;
    private boolean closedLoop = false;

    public IntakeIOSim() {
        motor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.001);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = motor.getOutput(0);
        inputs.velocityRPM = motor.getAngularVelocityRPM();

        if (closedLoop) {
            voltage = MathUtil.clamp(feedback.calculate(motor.getAngularVelocityRPM()) + ffVoltage, -12.0, 12.0);
            motor.setInputVoltage(voltage);
        }
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        feedback.setP(kp);
        feedback.setI(ki);
        feedback.setD(kd);
    }

    public void setVelocity(double rpm, double ffVoltage) {
        closedLoop = true;
        this.ffVoltage = ffVoltage;
        feedback.setSetpoint(rpm);
    }

    @Override
    public void setVoltage(double voltage) {
        closedLoop = false;
        motor.setInputVoltage(voltage);
    }
}
