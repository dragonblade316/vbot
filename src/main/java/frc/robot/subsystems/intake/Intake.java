package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private SimpleMotorFeedforward feedforward;
    
    private double setpointRPM = 0;
    private boolean closedLoopControl = true;

    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private SysIdRoutine sysId;

    //I just had this thought earlier today. By monitoring current/voltage spikes we may be able to detect when the intake jams and automatically attempt to fix it
    private boolean isjammed = false;

    

    public Intake(IntakeIO io) {
        switch (Constants.currentMode) {
            case REAL:
                feedforward = new SimpleMotorFeedforward(0.32269, 0.010463);
                io.setPID(8.9256E-07, 0.00, 0);
                
                break;
            case REPLAY:
                feedforward = new SimpleMotorFeedforward(0, 0);
                
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 0);
                io.setPID(0.001, 0, 0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0.0, 0);
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
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
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

    public void IntakeIn() {
        setpointRPM = -600;
        io.setVelocity(setpointRPM, feedforward.calculate(setpointRPM));
    }

    public void IntakeOut() {
        setpointRPM = 300;
        io.setVelocity(setpointRPM, feedforward.calculate(setpointRPM));
    }

    public void stop() {
        setpointRPM = 0;
        io.setVelocity(setpointRPM, feedforward.calculate(setpointRPM));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/SetPointRPM", setpointRPM);
    }
}

