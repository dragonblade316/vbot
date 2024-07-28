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

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotState.FlywheelState;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveMode;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.extender.ExtenderIO;
import frc.robot.subsystems.extender.ExtenderIOSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.carrier.Carrier;
import frc.robot.subsystems.rollers.carrier.CarrierIO;
import frc.robot.subsystems.rollers.carrier.CarrierIOSim;
import frc.robot.subsystems.rollers.carrier.CarrierIOSparkMax;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.rollers.intake.IntakeIOSparkMax;
import frc.robot.util.vlib.FieldUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Arm arm;
    // private final Climber climber;
  private final Extender extender;
  //rollers subsystem and its components
  private final Rollers rollers;
  private final Intake intake;
  private final Carrier carrier;

  private final Superstructure superstructure = new Superstructure();



  // Controller
  private final Joystick rjoy = new Joystick(0);
  private final Joystick ljoy = new Joystick(1);
  private final Joystick buttonPanel = new Joystick(2);

  JoystickButton intakeButton;
  JoystickButton smartFireButton;
  JoystickButton autoAmpButton;

  JoystickButton temp1;
  JoystickButton temp2;
  JoystickButton temp3;
  
  

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Driver> driverChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                // new ModuleIOSparkMax(0),
                // new ModuleIOSparkMax(1),
                // new ModuleIOSparkMax(2),
                // new ModuleIOSparkMax(3))
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        arm = new Arm(new ArmIOSim() {});
        extender = new Extender(new ExtenderIO() {});
        intake = new Intake(new IntakeIOSparkMax());
        carrier = new Carrier(new CarrierIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        arm = new Arm(new ArmIOSim());
        extender = new Extender(new ExtenderIOSim());
        intake = new Intake(new IntakeIOSim());   
        carrier = new Carrier(new CarrierIOSim());
        break;

      default:
        // Replayed robot, disable IOdawd implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        arm = new Arm(new ArmIO() {});
        extender = new Extender(new ExtenderIO() {});
        intake = new Intake(new IntakeIO() {});
        carrier = new Carrier(new CarrierIO() {});
        break;
    }

    rollers = new Rollers(intake, carrier);

    driverChooser = new LoggedDashboardChooser<>("Driver");
    driverChooser.addDefaultOption("Default", Driver.DEFAULT);
    
    
    // Set up auto routines
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
    //         .withTimeout(5.0));
    //autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic Forward)",
    //     flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic Reverse)",
    //     flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Flywheel SysId", flywheel.sysidCommand());
    autoChooser.addOption(
        "Rollers SysID", rollers.sysIdCommand());

    autoChooser.addOption("chortest", Autos.test(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //set per driver bindings here later
    defaultDriverBindings();
    
    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.updateTeleopInputs(rjoy.getY(), rjoy.getX(), -ljoy.getX()), 
            drive
        )
    );

    intakeButton.whileTrue(rollers.setGoalCommand(Rollers.Goal.Intake));
    autoAmpButton.whileTrue(drive.setHeadingCommand(() -> FieldUtils.apply(Rotation2d.fromDegrees(90))).alongWith(arm.setGoalCommand(Arm.SetGoal.AMP)).alongWith(RobotState.get_instance().AmpTrapModeCommand()));
    smartFireButton.whileTrue(
      Commands.either(
        drive.setHeadingWithTranslationCommand(RobotState.AimingFunctions.heading)
        .alongWith(arm.setGoalCommand(RobotState.AimingFunctions.armAngle))
        .alongWith(flywheel.setVelocityCommand(() -> 2000))
        ,
        
        Commands.none(), 
        
        () -> RobotState.get_instance().smartFireMode == RobotState.SmartFireMode.Standard));

    //temp1.whileTrue(arm.setGoalCommand(Arm.SetGoal.SPEAKER_DEAD_REKON));
    //temp1.whileTrue(flywheel.setVelocityCommand(() -> 1000));

    temp1.whileTrue(
      new InstantCommand(() -> flywheel.setVelocity(() -> 2000)).until(() -> RobotState.get_instance().shooterFlywheelState == FlywheelState.READY).andThen(() -> rollers.setGoal(Rollers.Goal.Shoot))
    );
    temp2.whileTrue(rollers.setGoalCommand(Rollers.Goal.Barf));
    

  }

  private enum Driver {
    DEFAULT,
    SIM,
  }

  public void perDriveBinder() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    switch (driverChooser.get()) {
        case DEFAULT:
            defaultDriverBindings();
            break;

        case SIM:
            simDriverBindings();
            break;
    
        default:
            defaultDriverBindings();
            break;
    }
  }


  private void defaultDriverBindings() {
    intakeButton = new JoystickButton(rjoy, 1);
    autoAmpButton = new JoystickButton(rjoy, 2);
    smartFireButton = new JoystickButton(rjoy, 3);

    temp1 = new JoystickButton(ljoy, 1);
    temp2 = new JoystickButton(ljoy, 2);
  }

  private void simDriverBindings() {
    intakeButton = new JoystickButton(ljoy, 1);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  //init functions, called by Robot.java
  public void auto() {
    drive.setMode(DriveMode.Auto);
  }

  public void teleop() {
    drive.setMode(DriveMode.Teleop);
  }
}
