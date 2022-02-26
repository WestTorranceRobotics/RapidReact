// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveTrain.JoystickTankDrive;
import frc.robot.commands.Elevator.LiftBackwards;
import frc.robot.commands.Elevator.LiftDown;
import frc.robot.commands.Elevator.LiftForwards;
import frc.robot.commands.Elevator.LiftUp;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.ToggleIntakeDeploy;
import frc.robot.commands.shooter.*;
import frc.robot.commands.VisionProcessing.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static final Joystick driverLeft = new Joystick(0);
  public static final Joystick driverRight = new Joystick(1);
  public XboxController operator = new XboxController(2);

  public JoystickButton operatorA = new JoystickButton(operator, 1);
  public JoystickButton operatorB = new JoystickButton(operator, 2);
  public JoystickButton operatorX = new JoystickButton(operator, 3);
  public JoystickButton operatorY = new JoystickButton(operator, 4);
  public JoystickButton operatorLB = new JoystickButton(operator, 5);
  public JoystickButton operatorRB = new JoystickButton(operator, 6);
  public JoystickButton operatorLT = new JoystickButton(operator, 7);
  public JoystickButton operatorRT = new JoystickButton(operator, 8);
  public JoystickButton operatorBack = new JoystickButton(operator, 9);
  public JoystickButton operatorStart = new JoystickButton(operator,10);
  public JoystickButton operatorTest = new JoystickButton(operator, 9);
  public JoystickButton operatorStickLeft = new JoystickButton(operator, 11);
  public JoystickButton operatorStickRight = new JoystickButton(operator, 12);

  public JoystickButton driverRightTrigger = new JoystickButton(driverRight, 1);
  public JoystickButton driverRightThumb  = new JoystickButton(driverRight, 2);

  public POVButton operatorUp = new POVButton(operator, 0);
  public POVButton operatorDown = new POVButton(operator, 180);
  public POVButton operatorRight = new POVButton(operator, 90);
  public POVButton operatorLeft = new POVButton(operator, 270);

  public ShuffleboardTab display;

  //Subsystem
  private Shooter shooter;
  private DriveTrain driveTrain;
  private Intake intake;
  private Elevator elevator;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the button bindings
    configureSubsystems();
    configureDefaultCommands();
    configureButtonBindings();
    configureShuffleboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureShuffleboard(){
    ShuffleboardTab display = Shuffleboard.getTab("RobotVision");
    display.addNumber("hi", RobotContainer::x);

    display.addNumber("Left Encoder", driveTrain::getLeftEncoderTicks).withPosition(2, 2);
    display.addNumber("Right Encoder", driveTrain::getRightEncoderTicks).withPosition(3, 2);
    // display.addBoolean("IN CENTER", RobotContainer::isCenter);

  }

  private static boolean isCenter(){
    return (NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) < 200 &&
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) > 130);
  }

  private static double x(){
    return NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0);
  }

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new JoystickTankDrive(driverLeft, driverRight, driveTrain));

  }
  private void configureButtonBindings() {
    //Shooter
    operatorA.whenHeld(new ShootBallBasedOnRPM(shooter, 5700));

    //Intake
    operatorX.whenHeld(new RunIntake(intake));
    operatorY.whenHeld(new ReverseIntake(intake));
    operatorBack.whenPressed(new ToggleIntakeDeploy(intake));

    //Elevator
    operatorUp.whileHeld(new LiftUp(elevator));
    operatorDown.whileHeld(new LiftDown(elevator));
    operatorLeft.whileHeld(new LiftForwards(elevator));
    operatorRight.whileHeld(new LiftBackwards(elevator));

    //Vision
    operatorLT.whenPressed(new visiondriving(driveTrain));
  }

  private void configureSubsystems(){
    shooter = new Shooter();
    driveTrain = new DriveTrain();
    intake = new Intake();
    elevator = new Elevator();
  }  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An ExampleCommand will run in autonomous
    return new ShootBallBasedOnPower(shooter, 0.5);
  }
}
