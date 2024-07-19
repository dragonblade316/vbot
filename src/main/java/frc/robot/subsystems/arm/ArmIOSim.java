package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

//This is not ready to go
public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim motor;
    private double voltage = 0;

    public ArmIOSim() {
        motor = new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          1.06328, //TODO: calculate this
          1.5,
          Units.degreesToRadians(15),
          Units.degreesToRadians(84),
          false,
          Units.degreesToRadians(17));
    }


    @Override
    public void updateInputs(ArmIOInputs inputs) {
        motor.update(0.02);

        inputs.angle = Rotation2d.fromRadians(motor.getAngleRads());
        inputs.velocityRadPerSec = motor.getVelocityRadPerSec();
        inputs.appliedVolts = voltage;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setInputVoltage(voltage);
    }

}
