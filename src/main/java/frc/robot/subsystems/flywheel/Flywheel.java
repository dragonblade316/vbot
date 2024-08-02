// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.FlywheelState;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private DoubleSupplier velocityRPM = () -> 0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;



    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL, REPLAY:
        ffModel = new SimpleMotorFeedforward(0.26202, 0.0020314);
        io.configurePID(2.3474E-06, 0.0, 0.0);
        break;
      // case REPLAY:
      //   ffModel = new SimpleMotorFeedforward(0.1, 0.05);
      //   io.configurePID(1.0, 0.0, 0.0);
      //   break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.4, 0.0);
        io.configurePID(0.01, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    //TODO: change the ramp rate so it does not wreak the flywheels
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    
    if (Math.abs(inputs.velocityRPM - velocityRPM.getAsDouble()) < 100 && velocityRPM.getAsDouble() != 0) {
      RobotState.get_instance().shooterFlywheelState = FlywheelState.READY;
    } else if (velocityRPM.getAsDouble() != 0) {
      RobotState.get_instance().shooterFlywheelState = FlywheelState.ACCELERATING;
    } else {
      RobotState.get_instance().shooterFlywheelState = FlywheelState.INACTIVE;
    }
    Logger.recordOutput("Flywheel/state", RobotState.get_instance().shooterFlywheelState);

    // Log flywheel setpoin to one of the labelet
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM.getAsDouble());

    io.setVelocity(velocityRPM.getAsDouble(), ffModel.calculate(velocityRPM.getAsDouble()));
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(DoubleSupplier velocityRPM) {
    this.velocityRPM = velocityRPM;
  }

  public Command setVelocityCommand(DoubleSupplier velocityRPM) {
    return startEnd(() -> setVelocity(velocityRPM), () -> setVelocity(() -> 0));
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command sysidCommand() {
    return sysId.quasistatic(Direction.kForward).andThen(
      Commands.waitSeconds(1),
      sysId.quasistatic(Direction.kReverse),
      Commands.waitSeconds(1),
      sysId.dynamic(Direction.kForward),
      Commands.waitSeconds(1),
      sysId.dynamic(Direction.kReverse)
    );
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRPM);
  }
}
