package frc.robot.subsystems.carrier;


import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CarrierIOSim implements CarrierIO {
   //SimDeviceSim sensorSim = new SimDeviceSim("CarrierPieceDetection");
   DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.001);
   PIDController feedback = new PIDController(0, 0, 0);

   boolean closedLoop = false;
   double ffVoltage = 0;
   double voltage = 0;

   public CarrierIOSim() {
        //SimDouble field 
        //sensorSim.getBoolean("isPiecePresent").set(false);
   }

    @Override
    public void updateInputs(CarrierIOInputs inputs) {
        inputs.isPiecePresent = false; //sensorSim.getBoolean("isPiecePresent").get();
        inputs.appliedVolts = voltage;
        inputs.velocityRPM = motor.getAngularVelocityRPM();

        if (closedLoop) {
            voltage = MathUtil.clamp(feedback.calculate(motor.getAngularVelocityRadPerSec()) + ffVoltage, -12.0, 12.0);
            motor.setInputVoltage(voltage);
        }
    }

    @Override
    public void setVelocity(double rpm, double ffVoltage) {
        closedLoop = true;
        this.ffVoltage = ffVoltage;
        feedback.setSetpoint(ffVoltage);
    }

    @Override
    public void setVoltage(double voltage) {
        closedLoop = false;
        this.voltage = voltage;
        motor.setInputVoltage(voltage);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        feedback.setPID(kp, ki, kd);
    }
}
