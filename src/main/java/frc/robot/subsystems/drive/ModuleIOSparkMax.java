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

package frc.robot.subsystems.drive;

import java.util.OptionalDouble;
import java.util.Queue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  //public static final double DRIVE_GEAR_RATIO = 1 / 7.13;
  //public static final double WHEEL_DIAMITER_INCH = 3.9375; // was 4
  //public static final double WHEEL_DIAMETER_METERS = 0.1000125;
  // public static final double WHEEL_DIAMITER_INCH = 4; // was 4
  // public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMITER_INCH / 39.37;
  // public static final double DRIVE_ENCODER_ROT_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
  // public static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROT_TO_METER / 60; 

  // TODO: change this
  private static final double DRIVE_GEAR_RATIO = (1 / 7.13);
  private static final double TURN_GEAR_RATIO = 1;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> driveVelocityQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor = new MagnetSensorConfigs();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    switch (index) {
      case 0:
        turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        driveSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(1);

        config.MagnetSensor.MagnetOffset = 0.396728515625;
        turnAbsoluteEncoder.getConfigurator().apply(config);

        driveSparkMax.setInverted(true);

        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        driveSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(2);

        config.MagnetSensor.MagnetOffset = -0.3564453125;
        turnAbsoluteEncoder.getConfigurator().apply(config);

        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        driveSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(3);

        config.MagnetSensor.MagnetOffset = -0.46142578125;
        turnAbsoluteEncoder.getConfigurator().apply(config);

        driveSparkMax.setInverted(true);

        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        driveSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(4);

        config.MagnetSensor.MagnetOffset = 0.496826171875;
        turnAbsoluteEncoder.getConfigurator().apply(config);

        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();



    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  //System.out.println("getting pos");
                  double value = driveEncoder.getPosition();
                  if (driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    driveVelocityQueue = 
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
              () -> {
                //System.out.println("getting vel");
                double value = driveEncoder.getVelocity();
                if (driveSparkMax.getLastError() == REVLibError.kOk) {
                  return OptionalDouble.of(value);
                } else {
                  return OptionalDouble.empty();
                }
              });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  //I know this is not the proper way to do thing but idk the proper way so this will work.(hopefully I will not accidently destroy the can bus utilization (but then again this project was going to do that anyway))
                  double value = turnAbsoluteEncoder.getAbsolutePosition().getValue();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(
                turnAbsoluteEncoder.getPosition().getValue())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryDriveVelocitiesRadPerSecond = 
        driveVelocityQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsPerMinuteToRadiansPerSecond(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    driveVelocityQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
