package frc.robot.subsystems.rollers.intake;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.GenericRollers;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.util.misc.ArrayCircularQueue;

public class Intake implements GenericRollers<Intake.IntakeGoal> {
    private IntakeIO io;
    private SimpleMotorFeedforward feedforward;
    
    private double setpointRPM = 0;
    private boolean closedLoopControl = true;

    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    //I just had this thought earlier today. By monitoring current/voltage spikes we may be able to detect when the intake jams and automatically attempt to fix it
    private boolean isjammed = false;
    private ArrayCircularQueue<Double> currentBuffer = new ArrayCircularQueue<Double>(10, 0.0);

    public IntakeGoal goal = IntakeGoal.Stop;
    public enum IntakeGoal implements GenericRollers.Goal {
        Intake(600),
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
                feedforward = new SimpleMotorFeedforward(0.3, 0.01001);
                io.setPID(0.0001, 0, 0);
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);


        //<jam detection>
        currentBuffer.enQueue(inputs.currentAmps);
        double avg = 0;
        for(int i = 0; i < 10; i++) {
            avg = avg + currentBuffer.get(i);
        }
        avg = avg / 10;
        if (avg < 60) {
            isjammed = true;
        }
        Logger.recordOutput("Intake/IsJammed", isjammed);
        //</jam detection>

        io.setVelocity(goal.getRpmGoal(), feedforward.calculate(goal.getRpmGoal()));
        
        Logger.recordOutput("Intake/SetPointRPM", goal.getRpmGoal());
    }
}

