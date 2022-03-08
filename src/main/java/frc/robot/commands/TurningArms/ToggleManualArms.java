// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurningArms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ElevatorMap;
import frc.robot.subsystems.TurningArms;

public class ToggleManualArms extends CommandBase {
  double leftPower = 0;
  double rightPower = 0;
  TurningArms turningArms;
  XboxController operator;
  /** Creates a new ToggleManualArms. */
  public ToggleManualArms(TurningArms turningArms, XboxController controller) 
  {
    this.turningArms = turningArms;
    operator = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.turningArms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // if(controller.getRightBumper())
    // {
    //   leftPower = RobotMap.ElevatorMap.elevatorMotorDown;
    // }
    // // else if(controller.getRightTriggerAxis() < 0)
    // // {
    // //   rightPower = RobotMap.ElevatorMap.elevatorMotorUp;
    // // }
    // else{ rightPower = 0; }

    // if(controller.getLeftBumper())
    // {
    //   leftPower = RobotMap.ElevatorMap.elevatorMotorDown;
    // }
    // // else if(controller.getLeftTriggerAxis() < 0)
    // // {
    // //   leftPower = RobotMap.ElevatorMap.elevatorMotorUp;
    // // }
    // else{ leftPower = 0; }

    // turningArms.ManualControl(leftPower, rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
