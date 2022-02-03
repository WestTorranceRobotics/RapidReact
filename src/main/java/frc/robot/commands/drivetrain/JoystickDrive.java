// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class JoystickDrive extends CommandBase {
  // Define/Create your variables for your commands like variables for subsystems and constants

   //defining subsystems. (Needed to actually use in methods)
  DriveTrain driveTrain;

  //defining Joysticks. (Needed to actually drive the robot)
  Joystick leftJoystick;
  Joystick rightJoystick;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(DriveTrain driveTrain, Joystick leftJoystick, Joystick rightJoystick) {
    this.driveTrain = driveTrain;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick; 

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!driveTrain.isAutomatic()) {
        if (leftJoystick.getY() > 0.1 || rightJoystick.getY() > 0.1 || rightJoystick.getY() < -0.1 || leftJoystick.getY() < -0.1){
            driveTrain.tankDrive(-leftJoystick.getY(), -rightJoystick.getY());
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}