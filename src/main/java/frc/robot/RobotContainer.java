// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.shooter.TurnToAngleUsingLimelight;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ShuffleboardTab display;
  private DriveTrain driveTrain;

  public static final Joystick driverLeft = new Joystick(0);
  public static final Joystick driverRight = new Joystick(1);
  public XboxController operator = new XboxController(2);
  
  public JoystickButton operatorA = new JoystickButton(operator, 2);
  public JoystickButton trigger = new JoystickButton(driverRight, 1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureSubsystems();
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
    // operatorA.toggleWhenPressed(new TurnToAngleUsingLimelight(driveTrain));
    trigger.toggleWhenPressed(new TurnToAngleUsingLimelight(driveTrain));
  }

  private void configureSubsystems(){
    driveTrain = new DriveTrain();
  }
  
  private void configureShuffleboard(){
    CameraServer.startAutomaticCapture();
    display = Shuffleboard.getTab("Driving Display");
    Shuffleboard.selectTab(display.getTitle());

    NetworkTableEntry pipeEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");

    display.addBoolean("Limelight On?",() -> (int) pipeEntry.getDouble(-1) == 0)
    .withPosition(4, 1).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);


    display.addNumber("kP", driveTrain::getP)
    .withPosition(3, 2).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);
    
    display.addNumber("kI", driveTrain::getI)
    .withPosition(4, 2).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);

    display.addNumber("kD", driveTrain::getD)
    .withPosition(5, 2).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
