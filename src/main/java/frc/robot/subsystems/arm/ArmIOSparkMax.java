package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmIOSparkMax implements ArmIO {
    CANSparkMax moter = new CANSparkMax(1, MotorType.kBrushless);
    RelativeEncoder rEncoder;
    CANcoder encoder = new CANcoder(1);

    public ArmIOSparkMax() {
        rEncoder = moter.getEncoder();

        //add abs encoder offset and r encoder conversion factor
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(encoder.getPosition().getValue()); //TODO: offset.
        inputs.appliedVolts = moter.getAppliedOutput();
        //why is rev so much nicer to work with then ctre when they can not even get their sims straight
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rEncoder.getVelocity());
    }

    @Override
    public void setVoltage(double voltage) {
        moter.setVoltage(voltage);
    }
}