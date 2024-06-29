package frc.robot.subsystems.carrier;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CarrierIOSim implements CarrierIO {
   SimDeviceSim sensorSim = new SimDeviceSim("CarrierPieceDetection");
   DCMotorSim moter = new DCMotorSim(DCMotor.getNEO(1), 1, 0.001);
   double voltage = 0;

   public CarrierIOSim() {
       sensorSim.getBoolean("isPiecePresent");
   }

    @Override
    public void updateInputs(CarrierIOInputs inputs) {
        inputs.isPiecePresent = sensorSim.getBoolean("isPiecePresent").get();
        inputs.appliedVolts = voltage;
        inputs.velocity = moter.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double voltage) {
       this.voltage = voltage;
       moter.setInputVoltage(voltage);
    }
}
