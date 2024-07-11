// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
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
      IR.quasistatic(Direction.kReverse),
      IR.dynamic(Direction.kForward),
      IR.dynamic(Direction.kReverse),
      CR.quasistatic(Direction.kForward),
      CR.quasistatic(Direction.kReverse),
      CR.dynamic(Direction.kForward),
      CR.dynamic(Direction.kReverse)

    );
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (goal == Goal.Intake && state.containsPiece) {
      goal = Goal.Stop;
    }

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
      default:
        intake.setGoal(IntakeGoal.Stop);
        carrier.setGoal(CarrierGoal.Stop);
        break;
    }
  }
}
