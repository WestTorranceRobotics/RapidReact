// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class LiftDown extends CommandBase {
  boolean isFinished = false;
  //creates new elevator
  private final Elevator elevator;

  public LiftDown(Elevator subsystem) {
    elevator = subsystem;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.liftDown();
    //when started up, elevator will go down
    // if (elevator.resetMode) {
    //   elevator.liftDown();
    // }
    // else if (elevator.getElevatorMotor().getEncoder().getPosition() <= RobotMap.ElevatorMap.elevatorMinHeight){
    //   elevator.getElevatorMotor().set(0);
    //   isFinished = true;
    // }
    // else {
    //   elevator.liftDown();
    // }
    // System.out.println(elevator.getElevatorMotor().getEncoder().getPosition() <= RobotMap.ElevatorMap.elevatorMinHeight);
    // System.out.println(isFinished);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //when command ends, no power is sent to the elevator
    elevator.setNoPower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
