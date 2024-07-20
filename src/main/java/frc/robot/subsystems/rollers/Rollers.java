// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.subsystems.rollers.carrier.Carrier;
import frc.robot.subsystems.rollers.carrier.Carrier.CarrierGoal;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.Intake.IntakeGoal;

public class Rollers extends SubsystemBase {
  public enum Goal {
    Stop,
    Intake,
    Shoot,
    Barf,
  }
  private Goal goal = Goal.Stop;

  private Intake intake;
  private Carrier carrier;

  //just bc its easier to type
  private RobotState state = RobotState.get_instance();

  /** Creates a new rollers. */
  public Rollers(Intake intake, Carrier carrier) {
    this.intake = intake;
    this.carrier = carrier;
  }

  public Command sysIdCommand() {
    var IR = intake.getSysIdRoutine(this);
    var CR = carrier.getSysIdRoutine(this);

    return IR.quasistatic(Direction.kForward).andThen(
      Commands.waitSeconds(1),
      IR.quasistatic(Direction.kReverse),
      Commands.waitSeconds(1),
      IR.dynamic(Direction.kForward),
      Commands.waitSeconds(1),
      IR.dynamic(Direction.kReverse),
      Commands.waitSeconds(1),
      CR.quasistatic(Direction.kForward),
      Commands.waitSeconds(1),
      CR.quasistatic(Direction.kReverse),
      Commands.waitSeconds(1),
      CR.dynamic(Direction.kForward),
      Commands.waitSeconds(1),
      CR.dynamic(Direction.kReverse)

    );
  }

  public void setGoal(Goal goal) {
    Logger.recordOutput("rollers/goal", goal);
    this.goal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.Stop), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intake.periodic();
    carrier.periodic();


    if (goal == Goal.Intake && state.containsPiece) {
      goal = Goal.Stop;
    }

    if (intake.isJammed()) {
      goal = Goal.Barf;
    }

    Logger.recordOutput("rollers/goal", goal);

    switch (goal) {
      case Stop:
        intake.setGoal(IntakeGoal.Stop);
        carrier.setGoal(CarrierGoal.Stop);
        break;
      case Intake:
        intake.setGoal(IntakeGoal.Intake);
        carrier.setGoal(CarrierGoal.Intake);
        break;
      case Shoot:
        intake.setGoal(IntakeGoal.Stop);
        carrier.setGoal(CarrierGoal.Shoot);
        break;
      case Barf:
        //the robot can not barf unless its moving so this solves that issue
        if (RobotState.get_instance().poseEstimator.getRobotReletiveVelocity().vyMetersPerSecond < -1) {
          intake.setGoal(IntakeGoal.Barf);
          carrier.setGoal(CarrierGoal.Barf);
        } else {
          intake.setGoal(IntakeGoal.Stop);
          carrier.setGoal(CarrierGoal.Stop);
        }
      default:
        intake.setGoal(IntakeGoal.Stop);
        carrier.setGoal(CarrierGoal.Stop);
        break;
    }
  }
}
