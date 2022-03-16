// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.loader.ReverseLoader;
import frc.robot.commands.loader.RunLoader;
import frc.robot.commands.loader.SeeBallRunLoader;
import frc.robot.commands.auto.DriveOffAimAndShootTwoBalls;
import frc.robot.commands.auto.ShootAndDriveOff;
import frc.robot.commands.driveTrain.JoystickTankDrive;
import frc.robot.commands.elevator.LiftDown;
import frc.robot.commands.elevator.LiftUp;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetStartingPosition;
import frc.robot.commands.intake.UndeployIntake;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static final Joystick driverLeft = new Joystick(1);
  public static final Joystick driverRight = new Joystick(0);
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
  public JoystickButton operatorStart = new JoystickButton(operator, 10);
  public JoystickButton operatorStickLeft = new JoystickButton(operator, 11);
  public JoystickButton operatorStickRight = new JoystickButton(operator, 12);

  public JoystickButton driverRightTrigger = new JoystickButton(driverRight, 1);
  public JoystickButton driverRightThumb  = new JoystickButton(driverRight, 2);
  public JoystickButton driverRightButton3 = new JoystickButton(driverRight, 3);
  public JoystickButton driverRightButton4 = new JoystickButton(driverRight, 4);
  public JoystickButton driverRightButton5 = new JoystickButton(driverRight, 5);

  public JoystickButton driverLeftTrigger = new JoystickButton(driverLeft, 1);
  public JoystickButton driverLeftButton3 = new JoystickButton(driverLeft, 3);
  public JoystickButton driverLeftButton4 = new JoystickButton(driverLeft, 4);
  public JoystickButton driverLeftButton5 = new JoystickButton(driverLeft, 5);

  public POVButton operatorUp = new POVButton(operator, 0);
  public POVButton operatorDown = new POVButton(operator, 180);
  public POVButton operatorRight = new POVButton(operator, 90);
  public POVButton operatorLeft = new POVButton(operator, 270);

  public ShuffleboardTab display;
  private HashMap<String, Command> autonomousCommandHashMap = new HashMap<>();
  private SendableChooser<String> autoSelector = new SendableChooser<String>();

  //private Shooter shooter;
  private DriveTrain driveTrain;
  private Intake intake;
  private Elevator elevator;
  private Loader loader;
  private TurningArms turningArms;
  private TestShooter testShooter;

  Timer timer = new Timer();

  public RobotContainer() {
    configureSubsystems();
    configureDefaultCommands();
    configureButtonBindings();
    configureShuffleboard();
  }

  private void configureShuffleboard() {
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("rpm").setDouble(0);
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("speed").setDouble(0);

    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootP").setDouble(0);
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootI").setDouble(0);
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootD").setDouble(0);
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootF").setDouble(0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(0);

    ShuffleboardTab display = Shuffleboard.getTab("RobotVision");
    Shuffleboard.selectTab("RobotVision");

    // display.addNumber("hi", () -> NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0));

    // display.addNumber("Left Encoder", driveTrain::getLeftEncoderTicks).withPosition(8, 2);
    // display.addNumber("Right Encoder", driveTrain::getRightEncoderTicks).withPosition(9, 2);

    // display.addBoolean("IS DEPLOYED?", intake::isDeployed).withPosition(0, 1);
    display.addNumber("Elevator Height", elevator::getElevatorMotorTicks);

    // display.addNumber("Poteniometer", intake::getAnalogIntakeValue).withWidget(BuiltInWidgets.kGraph).withSize(3, 3);

    // display.addNumber("Current", shooter::getCurrent).withPosition(5, 0);
    // display.addNumber("Loader applied output", loader::getAppliedOutput).withPosition(6, 0);
    // // display.addNumber("Balls shot", shooter::getBallsShot).withPosition(7, 0);

    // // for finding the range of distances from target that we can shoot from
    // display.addNumber("Distance From Target", driveTrain::getDistanceFromTarget).withPosition(1, 1);

    // // gets current current of shooter
    // display.addNumber("Current current of shooter", shooter::getCurrent).withPosition(2, 1);
    // display.addNumber("Balls shot", shooter::getBallsShot).withPosition(3, 1);

    // display.addBoolean("Shooter at speed", shooter::atSpeed).withPosition(9, 0);

    // display.addNumber("Current angle", driveTrain::getAngle).withPosition(0, 4);

    // //display.addNumber("Applied Power on Shooter", shooter::getCurrent).withWidget(BuiltInWidgets.kGraph).withSize(3, 3);
    // display.addNumber("Velocity on Shooter", shooter::getVelocity).withWidget(BuiltInWidgets.kGraph).withSize(3, 3).withPosition(4, 1);
    // // display.addBoolean("IN CENTER", RobotContainer::isCenter);
    
    // Actual Driver Shuffleboard screen
    ShuffleboardTab screen = Shuffleboard.getTab("Divingstation");
    Shuffleboard.selectTab(screen.getTitle());
    
    // add limelight camera from cameraserver (might just have to open networktables tab and drag it in)
    // add intake camera as well (might also have to drag it in from network tables)

    configureAutonomousSelector(screen);
    screen.addBoolean("INTAKE DEPLOYED?", intake::isDeployed).withPosition(0, 1).withSize(2, 1);
    screen.addBoolean("INTAKE RUNNING?", intake::isRunning).withPosition(0, 2).withSize(2, 1);
    screen.addBoolean("LOADER RUNNING?", loader::isRunning).withPosition(0, 3).withSize(2, 1);
    //screen.addBoolean("SHOOTER RUNNING?", shooter::active).withPosition(0, 4).withSize(2, 1);
    screen.addNumber("TestShooter Leader Velocity", testShooter::getVelocityLeader);
    screen.addNumber("TestShooter Follower Velocity", testShooter::getVelocityFollower);
    screen.addBoolean("BOTTOM LIMIT HIT", () -> elevator.getElevatorMotor().getEncoder().getPosition() <= RobotMap.ElevatorMap.elevatorMinHeight)
    .withPosition(2, 0).withSize(2, 1);
    screen.addBoolean("TOP LIMIT HIT", () -> elevator.getElevatorMotor().getEncoder().getPosition() >= RobotMap.ElevatorMap.elevatorMaxHeight)
    .withPosition(2, 1).withSize(2, 1);

    screen.addNumber("TIME", this::getTimeRemaining).withPosition(2, 0);
  }

  public int getTimeRemaining() {
    return 150 - (int) Timer.getMatchTime();
  }

  private void configureAutonomousSelector(ShuffleboardTab display) {
    // autoSelector.addOption("1 Ball", "1 Ball");
    autoSelector.addOption("2 Ball", "2 Ball");
    autoSelector.addOption("Simple Drive Off", "Simple Drive Off");

    display.add("Auto Selector", autoSelector)
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
    
    // autonomousCommandHashMap.put("1 Ball", new DriveOffAimAndShootOneBall(driveTrain, intake, loader, shooter));
    // autonomousCommandHashMap.put("2 Ball", new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter));
    // autonomousCommandHashMap.put("ShootAndDriveOff", new ShootAndDriveOff(driveTrain, loader, intake, shooter));
  }

  // private static boolean isCenter(){
  //   return (NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) < 200 &&
  //   NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) > 130);
  // }

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new JoystickTankDrive(driverLeft, driverRight, driveTrain));
    // loader.setDefaultCommand(new SeeBallRunLoader(loader));
  }

  private void configureButtonBindings() {
    // driverRightButton3.whenHeld(new MoveBallFromShooterToIntake(intake, loader));

    // debug controls

    // driverRightThumb.whenHeld(new ParallelCommandGroup(new ReverseLoader(loader), new ReverseIntake(intake)));
    
    // driverRightButton3.toggleWhenPressed(new TurnToAngle(driveTrain, 90));
    // driverRightButton3.whenPressed(new ThreeBallsNearTarmac(driveTrain, intake, loader, shooter));

    // driverLeftButton4.toggleWhenPressed(new ShootOneBallUsingDirectPower(shooter, loader, 1, 2500));

    // driverLeftButton4.whenPressed(new ConditionalCommand(new UndeployIntake(intake), new DeployIntake(intake), intake::isDeployed));
    // driverLeftButton3.whenPressed(new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter));
    
    // Correct Controls
    // Joystick controls
    driverRightTrigger.whenHeld(new RunLoader(loader)); // only run load
    driverRightThumb.whenHeld(new RunIntake(intake)); // intake and load
    // driverLeftTrigger.whenHeld(new ParallelCommandGroup( // aim and start up shooter
    //   new StayOnTarget(driveTrain),
    //   new ShootBallBasedOnPower(shooter, 0.7)
    //   // new ShootBallBasedOnRPM(shooter, 5700)
    // ));

    // driverLeftButton5.whenHeld(new ShootBallBasedOnPower(shooter, 0.3)); // for lower goal just in case
    // driverRightButton3.whenHeld(new ParallelCommandGroup( // aim and start up shooter
    //   new StayOnTarget(driveTrain),
    //   new ShootBallBasedOnPower(shooter, 0.6)
    // ));
    // driverRightButton5.whenHeld(new ParallelCommandGroup( // aim and start up shooter
    //   new StayOnTarget(driveTrain),
    //   new ShootBallBasedOnPower(shooter, 1)
    // ));

    //Intake
    operatorBack.whenHeld(new ParallelCommandGroup(new ReverseLoader(loader), new ReverseIntake(intake)));
    operatorA.whenHeld(new RunIntake(intake));
    operatorB.whenPressed(new ConditionalCommand(new UndeployIntake(intake), new DeployIntake(intake), intake::isDeployed));
    operatorStart.whenPressed(new SetStartingPosition(intake));
    operatorBack.whenHeld(new ShootingUsingLQR(testShooter));

    //Loader
    operatorX.whenHeld(new RunLoader(loader));
    operatorY.whenHeld(new ReverseLoader(loader));
  
    //Elevator
    operatorUp.whileHeld(new LiftUp(elevator));
    operatorDown.whileHeld(new LiftDown(elevator));

    // Pivot Arms
    operatorRT.whenHeld(new InstantCommand(turningArms::leftArmForwards), false)
    .whenReleased(new InstantCommand(turningArms::stopLeftArm), false);

    operatorLT.whenHeld(new InstantCommand(turningArms::rightArmForwards), false)
    .whenReleased(new InstantCommand(turningArms::stopRightArm), false);

    operatorRB.whenHeld(new InstantCommand(turningArms::leftArmBackwards), false)
    .whenReleased(new InstantCommand(turningArms::stopLeftArm), false);

    operatorLB.whenHeld(new InstantCommand(turningArms::rightArmBackwards), false)
    .whenReleased(new InstantCommand(turningArms::stopRightArm), false);
  }

  private void configureSubsystems(){
    // shooter = new Shooter();
    driveTrain = new DriveTrain();
    intake = new Intake();
    elevator = new Elevator();
    loader = new Loader();
    turningArms = new TurningArms();
    testShooter = new TestShooter();
  }  

  public Command getAutonomousCommand() {
    // if (autoSelector.getSelected() == null) {
    //   return new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter);
    // }
    // return autonomousCommandHashMap.get(autoSelector.getSelected());
    //return new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter);
    return null;//new ShootingUsingLQR(testShooter);
  }
}
