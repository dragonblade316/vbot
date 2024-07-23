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

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Controllers.AutoController;
import frc.robot.subsystems.drive.Controllers.HeadingController;
import frc.robot.subsystems.drive.Controllers.TeleopController;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.vlib.swerve.VSwervePoseEstimator;

public class Drive extends SubsystemBase {
  

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;


  public enum DriveMode {
    Teleop,
    Auto,
    Auto_Set_Heading
  }

  private DriveMode currentDriveMode = DriveMode.Auto;
  private TeleopController teleopController = new TeleopController();
  private AutoController autoController = new AutoController();
  private HeadingController headingController = null;


  private SwerveDriveKinematics kinematics = DriveConstants.kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());


  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        (speeds) -> autoController.updateInputs(speeds),
        new HolonomicPathFollowerConfig(
            DriveConstants.MAX_LINEAR_SPEED, DriveConstants.DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;

    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }
      
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        moduleStates[moduleIndex] = modules[moduleIndex].getOdometryStates()[i];
      }
 
      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      RobotState.get_instance().poseEstimator.updateOdometry(new VSwervePoseEstimator.OdometryObservation(modulePositions, moduleStates, rawGyroRotation, sampleTimestamps[i]));
    }
    //RobotState.get_instance().robotPose = poseEstimator.getEstimatedPosition();
    Logger.recordOutput("Odometry/CurrentPosition", poseEstimator.getEstimatedPosition());
    Logger.recordOutput("Odometry/TCurrentPosition", RobotState.get_instance().poseEstimator.getEstimatedPose());

    //Note: the sceduler runs each subsystem's periodic method first. This is important so the heading controller does not get outdated data since it will be using the global state.
    //guess this does not matter anymore.

    Logger.recordOutput("Drive/DriveMode", currentDriveMode);

    ChassisSpeeds speeds = new ChassisSpeeds();
    switch (currentDriveMode) {
      case Teleop:
        speeds = teleopController.update();

        //saw a fun idea where a heading controller maintains the heading when there is no rotation input

        if (headingController != null) {
          speeds = headingController.update(speeds);
        }
        break;
      case Auto:
        //I am now realizing that pathplanner will just call run runVelocity so these drive modes are just to make sure the (dang it I just realized I'm an idiot and may need to make an autoController). 
        //speeds = headingController.update(speeds, Rotation2d.fromRadians(gyroInputs.yawPosition.getRadians() + (speeds.omegaRadiansPerSecond / 50)));
        speeds = autoController.update();
        
        break;
      case Auto_Set_Heading:
        //this mode will be used when manual control is desirable. This should not be used with pathplanner since pathplanner has its own heading control 
        autoController.update();
        if (headingController != null) {
          speeds = headingController.update(speeds);
        }
        break;
    }
    
    Logger.recordOutput("Drive/Speeds", speeds);
    

    runVelocity(speeds);
  }


  public void updateTeleopInputs(double x, double y, double omega) {
    teleopController.updateInputs(x, y, omega);
  }

  public void setHeading(Supplier<Rotation2d> heading) {
    headingController = new HeadingController(heading);
    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(heading.get()));
  }

  public void setHeadingWithTranslation(Supplier<Translation2d> translation) {
    Supplier<Rotation2d> angle = () -> Rotation2d.fromRadians(RobotState.get_instance().poseEstimator.getEstimatedPose().getTranslation().minus(translation.get()).getAngle().getRadians());
    headingController = new HeadingController(angle);
    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(angle.get()));
  }

  public void clearHeading() {
    headingController = null;
    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
  }

  public void setMode(DriveMode mode) {
    currentDriveMode = mode;
  }

  /**
   * Runs the drive at the desired velocity.:w
   
   *
   * @param speeds Speeds in meters/sec
   */
  private void runVelocity(ChassisSpeeds speeds) {

    //TODO: I might be able to reduce drift on the swerve. I also may just turn this into another controller.
    //https://docs.google.com/presentation/d/1oh7BnamwzvyQyYRKtTKzhHc1T9l5YmhzTlriG2_he6M/edit#slide=id.g2df8babd0fa_5_157


    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    //TODO: figure out the acceleration limits. 
    //accel limit (im stealing orbits accel limits for the time being)
    ChassisSpeeds wantedAcc = speeds.minus(kinematics.toChassisSpeeds(getModuleStates()));
    //wantedAcc = new ChassisSpeeds(DriveConstants.MAX_FORWARD_ACC, DriveConstants.MAX_FORWARD_ACC, DriveConstants.MAX_FORWARD_ACC / DriveConstants.DRIVE_BASE_RADIUS).times(1-kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond- DriveConstants.MAX_LINEAR_SPEED);
    


    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

    //velocity limit
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    RobotState.get_instance().poseEstimator.setPose(pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return DriveConstants.getModuleTranslations();
  }

  public Command setHeadingCommand(Supplier<Rotation2d> heading) {
    return Commands.startEnd(() -> setHeading(heading), () -> clearHeading());
  }

  public Command setHeadingWithTranslationCommand(Supplier<Translation2d> translation) {
    return Commands.startEnd(() -> setHeadingWithTranslation(translation), () -> clearHeading());
  }
}
