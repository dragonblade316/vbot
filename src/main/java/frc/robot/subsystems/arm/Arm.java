package frc.robot.subsystems.arm;


import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.armUtils.VArmFeedforward;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private VArmFeedforward feedforward;
    //TODO: fix the VPID and use here
    private PIDController feedback;

    private Supplier<Rotation2d> targetAngle = () -> Rotation2d.fromDegrees(18);
    private TrapezoidProfile.State setPointState;

    //remember, this is all in Rads Per Second.
    TrapezoidProfile profile = new TrapezoidProfile(new Constraints(1, 1));

    private static final int LOWEST_ANGLE = 12;
    private static final int HIGHEST_ANGLE = 74;

    private boolean climblock = false;

    //all of these are in degrees, they will be converted to rads
    public enum SetGoal {
        TRAVERSE(17),
        AMP(45),
        TRAP(43.5),
        SPEAKER_DEAD_REKON(50),
        CLIMB(73);

        public final Double angle;

        private SetGoal(double angle) {
            this.angle = angle;
        }
    }

    public Arm(ArmIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
                feedforward = new VArmFeedforward(0, 0, 0);
                feedback = new PIDController(0, 0, 0);
                break;
            
            case REPLAY:

                break;

            default:
                //these guessed values are good enough for now but may need more tuning in the future
                feedforward = new VArmFeedforward(3, 0, 3);
                feedback = new PIDController(2, 0, 0);
                break;
        }

        io.updateInputs(inputs);
        setPointState = new TrapezoidProfile.State(inputs.angle.getRadians(), inputs.velocityRadPerSec);
    }

    public void setGoal(Supplier<Rotation2d> angle) {
        targetAngle = angle;
    }

    public void setGoal(SetGoal position) {
        setGoal(() -> Rotation2d.fromDegrees(position.angle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        //I know this is confusing just bear with me.
        Supplier<Rotation2d> targetAngle = this.targetAngle;
        if (RobotState.get_instance().climbersUp) {
            targetAngle = () -> Rotation2d.fromDegrees(SetGoal.CLIMB.angle);
        }

        var setpointState = profile.calculate(
            0.02, 
            setPointState, new TrapezoidProfile.State(
            MathUtil.clamp(
                targetAngle.get().getRadians(),
                Units.degreesToRadians(LOWEST_ANGLE),
                Units.degreesToRadians(HIGHEST_ANGLE)),
            0.0));


        //TODO: I should probably add in safety limits or smtn but whatever
        io.setVoltage(feedback.calculate(inputs.angle.getRadians(), targetAngle.get().getRadians()) + feedforward.calculate(setpointState.position, setpointState.velocity));
    }
}

