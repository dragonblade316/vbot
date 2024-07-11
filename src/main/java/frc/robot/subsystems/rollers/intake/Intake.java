package frc.robot.subsystems.rollers.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.GenericRollers;

public class Intake implements GenericRollers<Intake.IntakeGoal> {
    private IntakeIO io;
    private SimpleMotorFeedforward feedforward;
    
    private double setpointRPM = 0;
    private boolean closedLoopControl = true;

    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    //I just had this thought earlier today. By monitoring current/voltage spikes we may be able to detect when the intake jams and automatically attempt to fix it
    private boolean isjammed = false;

    public IntakeGoal goal = IntakeGoal.Stop;
    public enum IntakeGoal implements GenericRollers.Goal {
        Intake(1000),
        Barf(-600),
        Stop(0)
        ;
        private double goal; 
        private IntakeGoal(double goal) {
            this.goal = goal;
        }
        public Double getRpmGoal() {
            return this.goal;
        }
    }

    

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

    }

    // @Override
    // public void setGoal(Goal goal) {
    //     this.goal = goal;
    // }

    public void setGoal(IntakeGoal goal) {
        this.goal = goal;
    }

    @Override
    public IntakeGoal getGoal() {
        return goal;
    }

    @Override
    public SysIdRoutine getSysIdRoutine(Rollers rollers) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, rollers));

    }

    @Override
    public void setVoltage(double voltage) {
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

        io.setVelocity(goal.getRpmGoal(), feedforward.calculate(goal.getRpmGoal()));
        
        Logger.recordOutput("Intake/SetPointRPM", setpointRPM);
    }
}

