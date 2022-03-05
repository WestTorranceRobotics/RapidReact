// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.commandGroups.AimAndShoot;
import frc.robot.commands.commandGroups.MoveBallFromIntakeToShooter;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.driveTrain.JoystickTankDrive;
import frc.robot.commands.elevator.LiftBackwards;
import frc.robot.commands.elevator.LiftDown;
import frc.robot.commands.elevator.LiftForwards;
import frc.robot.commands.elevator.LiftUp;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetStartingPosition;
import frc.robot.commands.intake.ToggleIntakeDeploy;
import frc.robot.commands.intake.UndeployIntake;
import frc.robot.commands.shooter.*;
import frc.robot.commands.visionProcessing.*;
import frc.robot.commands.loader.ReverseLoader;
import frc.robot.commands.loader.RunLoader;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static final Joystick driverLeft = new Joystick(0);
  public static final Joystick driverRight = new Joystick(1);
  public XboxController operator = new XboxController(2);

  public JoystickButton operatorX = new JoystickButton(operator, 1);
  public JoystickButton operatorA = new JoystickButton(operator, 2);
  public JoystickButton operatorB = new JoystickButton(operator, 3);
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

  public JoystickButton driverLeftTrigger = new JoystickButton(driverLeft, 1);
  public JoystickButton driverLeftThumb = new JoystickButton(driverLeft, 2);
  public JoystickButton driverLeftButton1 = new JoystickButton(driverLeft, 3);

  public POVButton operatorUp = new POVButton(operator, 0);
  public POVButton operatorDown = new POVButton(operator, 180);
  public POVButton operatorRight = new POVButton(operator, 90);
  public POVButton operatorLeft = new POVButton(operator, 270);

  public ShuffleboardTab display;

  private Shooter shooter;
  private DriveTrain driveTrain;
  private Intake intake;
  private Elevator elevator;
  private Loader loader;

  public RobotContainer() 
  {
    // Configure the button bindings
    configureSubsystems();
    configureDefaultCommands();
    configureButtonBindings();
    configureShuffleboard();
  }

  private void configureShuffleboard(){
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("rpm").setDouble(0);
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("speed").setDouble(0);

    NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootP").setDouble(0);
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootI").setDouble(0);
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootD").setDouble(0);
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootF").setDouble(0);
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("").setDouble(0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(0);

    ShuffleboardTab display = Shuffleboard.getTab("RobotVision");
    display.addNumber("hi", () -> NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0));

    display.addNumber("Left Encoder", driveTrain::getLeftEncoderTicks).withPosition(8, 2);
    display.addNumber("Right Encoder", driveTrain::getRightEncoderTicks).withPosition(9, 2);

    display.addBoolean("IS DEPLOYED?", intake::isDeployed).withPosition(0, 1);
    display.addNumber("Elevator Height", elevator::getElevatorMotorTicks);

    display.addNumber("Poteniometer", intake::getAnalogIntakeValue).withWidget(BuiltInWidgets.kGraph).withSize(3, 3);

    // for finding the range of distances from target that we can shoot from
    display.addNumber("Distance From Target", driveTrain::getDistanceFromTarget).withPosition(1, 1);

    // gets current current of shooter
    display.addNumber("Current current of shooter", shooter::getCurrent).withPosition(2, 1);
    display.addNumber("Balls shot", shooter::getBallsShot).withPosition(3, 1);

    //display.addNumber("Applied Power on Shooter", shooter::getCurrent).withWidget(BuiltInWidgets.kGraph).withSize(3, 3);
    display.addNumber("Velocity on Shooter", shooter::getVelocity).withWidget(BuiltInWidgets.kGraph).withSize(3, 3).withPosition(4, 1);
    // display.addBoolean("IN CENTER", RobotContainer::isCenter);
  }

  private static boolean isCenter(){
    return (NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) < 200 &&
    NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) > 130);
  }

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new JoystickTankDrive(driverLeft, driverRight, driveTrain));

  }
  private void configureButtonBindings() {
    //Shooter
    // driverRightTrigger.toggleWhenPressed(new StayOnTarget(driveTrain)); 
    driverRightTrigger.whenHeld(new MoveBallFromIntakeToShooter(loader, intake));
    driverRightThumb.whenHeld(new ReverseLoader(loader));
    driverLeftTrigger.whenHeld(new ShootBallBasedOnRPM(shooter, 3000));
    // driverLeftTrigger.whenHeld(new ShootBallBasedOnPower(shooter, 0));
    // driverLeftThumb.whenPressed(new DriveDistance(driveTrain, 24, 0.6));
    // driverLeftButton1.toggleWhenPressed(new ShootOneBallUsingDirectPower(shooter, loader));

    // driverLeftTrigger.whenPressed(new ParallelDeadlineGroup(
    //   new ShootOneBallUsingDirectPower(shooter, loader),
    //   new StayOnTarget(driveTrain),
    //   new MoveBallFromIntakeToShooter(loader, intake)
    // ));
    
    operatorRB.whenHeld(new ShootBallBasedOnRPM(shooter, 3000), true);
    
    //Intake
    operatorA.whenHeld(new RunIntake(intake));
    operatorY.whenHeld(new ReverseLoader(loader));

    operatorB.whenPressed(new ConditionalCommand(new UndeployIntake(intake), new DeployIntake(intake), intake::isDeployed));
    operatorStart.whenPressed(new SetStartingPosition(intake));

    //Elevator
    operatorUp.whileHeld(new LiftUp(elevator));
    operatorDown.whileHeld(new LiftDown(elevator));
    operatorLeft.whileHeld(new LiftForwards(elevator));
    operatorRight.whileHeld(new LiftBackwards(elevator));

    //Vision
    operatorLT.whenPressed(new ShootBallBasedOnRPM(shooter, 3000));
  }

  private void configureSubsystems(){
    shooter = new Shooter();
    driveTrain = new DriveTrain();
    intake = new Intake();
    elevator = new Elevator();
    loader = new Loader();
  }  

  public Command getAutonomousCommand() 
  {
    return new ShootBallBasedOnPower(shooter, 0.5);
  }
}
