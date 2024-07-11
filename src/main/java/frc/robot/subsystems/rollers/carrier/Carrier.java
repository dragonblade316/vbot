package frc.robot.subsystems.rollers.carrier;


import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.rollers.GenericRollers;
import frc.robot.subsystems.rollers.Rollers;

public class Carrier implements GenericRollers<Carrier.CarrierGoal> {
    private CarrierIO io;
    private SimpleMotorFeedforward feedforward;
    
    private double setpointRPM = 0;
    private boolean closedLoopControl = true;

    private CarrierGoal goal = CarrierGoal.Stop;
    public enum CarrierGoal implements GenericRollers.Goal {
        Intake(1),
        Barf(-1),
        Shoot(5),
        Stop(0)
        ;
        private double rpmGoal;
        private CarrierGoal(double rpmGoal) {
            this.rpmGoal = rpmGoal;
        }
        public Double getRpmGoal() {
            return this.rpmGoal;
        }
    }

    private Notifier notifier = new Notifier(null);


    private CarrierIOInputsAutoLogged inputs = new CarrierIOInputsAutoLogged();

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

        //run 100 time per second
        notifier.startPeriodic(0.01);

        this.io = io;
    }

    public void setGoal(CarrierGoal goal) {
        this.goal = goal;
    }

    public CarrierGoal getGoal() {
        return goal;
    }

    @Override
    public SysIdRoutine getSysIdRoutine(Rollers rollers) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Carrier/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, rollers));
    }

    @Override
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Carrier", inputs);

        if (goal == CarrierGoal.Intake && inputs.isPiecePresent) {
            goal = CarrierGoal.Stop;
        }

        RobotState.get_instance().containsPiece = inputs.isPiecePresent;

        io.setVelocity(goal.getRpmGoal(), feedforward.calculate(goal.getRpmGoal()));
        Logger.recordOutput("Intake/RPMGoal", goal.getRpmGoal());
    }

    public void checkPeriodic() {
        if (goal == CarrierGoal.Intake && inputs.isPiecePresent) {
            goal = CarrierGoal.Stop;
            periodic();
        }
    }

}

