// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class LiftUp extends CommandBase {

  //creates new elevator
  private final Elevator elevator;
  private boolean isFinished = false;

  public LiftUp(Elevator subsystem) {
    elevator = subsystem;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //when started up, elevator will lift up
    elevator.liftUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*this area is for checking if the climb is too high using encoders
      but the encoders originally used last year are from cansparkmax, not victorspx
      im planning to use gear ratios of elevatormotor to calculate this instead, may not be the case and will change accordingly later*/
      if(elevator.getElevatorMotor().getEncoder().getPosition() >= RobotMap.ElevatorMap.elevatorMaxHeight){
        elevator.setNoPower();
        isFinished = true;
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //when command ends, elevator stops
    elevator.setNoPower();
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
