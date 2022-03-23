// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap.DriveTrainMap;
import frc.robot.commands.JoystickTankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain driveTrain;
  private ShuffleboardTab display;
  private Intake intake;

  public static final Joystick driverLeft = new Joystick(0);
  public static final Joystick driverRight = new Joystick(1);
  public XboxController operator = new XboxController(2);
  
  public JoystickButton operatorA = new JoystickButton(operator, 2);
  public JoystickButton operatorB = new JoystickButton(operator, 3);
  public JoystickButton operatorX = new JoystickButton(operator, 1);
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
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
  private void configureButtonBindings() {

  }

  private void configureSubsystems(){
    driveTrain = new DriveTrain();
    intake = new Intake();
  }
  
  private void configureShuffleboard(){
    display = Shuffleboard.getTab("Driving Display");
    Shuffleboard.selectTab(display.getTitle());

    display.addNumber("Current Robot Angle", driveTrain::getAngle)
    .withPosition(3, 1).withSize(2, 2);

    display.addNumber("Current Robot left encoder", driveTrain::getLeftEncoderTicks)
    .withPosition(0, 1).withSize(2, 2);

    display.addNumber("Current Robot right encoder", driveTrain::getRightEncoderTicks)
    .withPosition(0, 3).withSize(2, 2);
  
    display.addNumber("Current Robot follower left encoder", driveTrain::getLeftFollowerEncoderTicks)
    .withPosition(7, 1).withSize(2, 2);

    display.addNumber("Current Robot follower right encoder", driveTrain::getRightFollowerEncoderTicks)
    .withPosition(7, 3).withSize(2, 2);

    display.addBoolean("Limit Swich works", intake::isActiviated);
  
    display.addNumber("Left Voltage", driveTrain::getVoltage);
  }


  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new JoystickTankDrive(driverLeft, driverRight, driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    DifferentialDriveVoltageConstraint autoVoltageConstraint = 
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
      RobotMap.DriveTrainMap.ksVolts, 
      RobotMap.DriveTrainMap.kvVoltSecondsPerMeter,
      RobotMap.DriveTrainMap.kaVoltSecondsSquaredPerMeter),
      RobotMap.DriveTrainMap.dKINEMATICS, 
      10);
    
    TrajectoryConfig config = 
    new TrajectoryConfig(
      RobotMap.DriveTrainMap.kMaxSpeedMetersPerSecond,
      RobotMap.DriveTrainMap.kMaxAccelerationMetersPerSecondSquared)
      .addConstraint(autoVoltageConstraint)
      .setKinematics(RobotMap.DriveTrainMap.dKINEMATICS);

      Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(0.5, 1), new Translation2d(1, 0)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(1.5, 0, new Rotation2d(0)),
          // Pass config
          config);
      
      RamseteCommand ramseteCommand =
        new RamseteCommand(exampleTrajectory,
         driveTrain::getPose2d, 
         new RamseteController(), 
         new SimpleMotorFeedforward(
        RobotMap.DriveTrainMap.ksVolts, 
        RobotMap.DriveTrainMap.kvVoltSecondsPerMeter,
        RobotMap.DriveTrainMap.kaVoltSecondsSquaredPerMeter),
        RobotMap.DriveTrainMap.dKINEMATICS,
        driveTrain::getWheelSpeeds,
        new PIDController(RobotMap.DriveTrainMap.kPDriveVel, 0, 0),
        new PIDController(RobotMap.DriveTrainMap.kPDriveVel, 0, 0),
         driveTrain::tankDriveVolts, 
         driveTrain);

         System.out.println("I AM RUNNING AUTO COMMAND");

    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
