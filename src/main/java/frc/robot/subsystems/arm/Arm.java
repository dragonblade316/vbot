package frc.robot.subsystems.arm;


import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.armUtils.VArmFeedforward;
import frc.robot.util.vlib.control.VPIDController;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private VArmFeedforward feedforward;
    //TODO: fix the VPID and use here
    private VPIDController feedback;

    private Supplier<Rotation2d> targetAngle = () -> Rotation2d.fromDegrees(18);
    private TrapezoidProfile.State setPointState;

    //remember, this is all in Rads Per Second.
    TrapezoidProfile profile = new TrapezoidProfile(new Constraints(1, 1));

    private static final int LOWEST_ANGLE = 12;
    private static final int HIGHEST_ANGLE = 74;

    private boolean climblock = false;

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("arm", 1, 0);
    MechanismLigament2d actual = new MechanismLigament2d("armCurrent", 0.8, 0, 6, new Color8Bit(Color.kRed));
    MechanismLigament2d target = new MechanismLigament2d("armTarget", 0.5, 0, 6, new Color8Bit(Color.kBlue));

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

        root.append(actual);
        root.append(target);

        switch (Constants.currentMode) {
            case REAL:
                feedforward = new VArmFeedforward(0, 0.1, 0);
                feedback = new VPIDController("ArmPID", 0, 0, 0);
                break;
            
            case REPLAY:

                break;

            default:
                //these guessed values are good enough for now but may need more tuning in the future
                feedforward = new VArmFeedforward(1.0, 0, 0);
                feedback = new VPIDController("ArmPID", 0, 0, 0);
                // feedback = new PIDController(4, 0, 0);
                break;
        }

        io.updateInputs(inputs);
        setPointState = new TrapezoidProfile.State(inputs.angle.getRadians(), 0);
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
            new TrapezoidProfile.State(
                inputs.angle.getRadians(),
                //this needs to be zero. for some reason the arm slowly ocillates between its highest and lowest points if this is the actual value
                0
            ), 
            new TrapezoidProfile.State(
            MathUtil.clamp(
                targetAngle.get().getRadians(),
                Units.degreesToRadians(LOWEST_ANGLE),
                Units.degreesToRadians(HIGHEST_ANGLE)),
            0.0));


        //TODO: I should probably add in safety limits or smtn but whatever
        io.setVoltage(feedback.calculate(inputs.angle.getRadians(), targetAngle.get().getRadians()) + feedforward.calculate(setpointState.position, setpointState.velocity));

        Logger.recordOutput("Arm/AtTarget", feedback.atSetpoint());
        Logger.recordOutput("Arm/TargetAngle", targetAngle.get());

        //visualization update

        actual.setAngle(inputs.angle);
        target.setAngle(targetAngle.get());
        Logger.recordOutput("Arm/MechState", mech);
    }

    public Command setGoalCommand(SetGoal goal) {
        return Commands.startEnd(() -> setGoal(goal), () -> setGoal(SetGoal.TRAVERSE), this);
    }

    public Command setGoalCommand(Supplier<Rotation2d> angle) {
        return Commands.startEnd(() -> setGoal(angle), () -> setGoal(SetGoal.TRAVERSE), this);
    }
}

