package frc.robot.subsystems.carrier;


import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Carrier extends SubsystemBase {
    private CarrierIO io;
    private SimpleMotorFeedforward feedforward;
    
    private double setpointRPM = 0;
    private boolean closedLoopControl = true;

    private CarrierIOInputsAutoLogged inputs = new CarrierIOInputsAutoLogged();
    private SysIdRoutine sysId;


    public Carrier(CarrierIO io) {
        switch (Constants.currentMode) {
            case REAL:
                feedforward = new SimpleMotorFeedforward(0.30474, 0.010524);
                io.setPID(6.0404E-07, 0, 0);
                break;
            case REPLAY:
                feedforward = new SimpleMotorFeedforward(0, 0);
                
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 0);
                io.setPID(1, 0, 0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0, 0);
                io.setPID(0, 0, 0);
                break;
        }

        this.io = io;

        sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Carrier/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    public void runVolts(double voltage) {
        io.setVoltage(voltage);
    }

    public void carrierIn() {
        setpointRPM = 600;
        io.setVelocity(600, feedforward.calculate(setpointRPM));
    }

    public void carrierOut() {
        setpointRPM = -100;
        io.setVelocity(-100, feedforward.calculate(setpointRPM));
    }

    public void carrierShoot() {
        setpointRPM = 400;
        io.setVelocity(400, feedforward.calculate(setpointRPM));
    }

    public void stop() {
        setpointRPM = 0;
        io.setVelocity(0, feedforward.calculate(setpointRPM));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Carrier", inputs);
    }

}

