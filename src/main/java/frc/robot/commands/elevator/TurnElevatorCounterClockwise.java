// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.subsystems.Elevator;

public class TurnElevatorCounterClockwise extends CommandBase {
  /** Creates a new TurnElevatorCounterClockwise. */
  private Elevator mElevator;
  public TurnElevatorCounterClockwise(Elevator elevator) {
    mElevator = elevator;

    addRequirements(mElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mElevator.turnLiftCounterClockwise(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mElevator.stopTurning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
