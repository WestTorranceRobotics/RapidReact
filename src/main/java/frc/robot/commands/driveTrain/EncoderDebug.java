// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class EncoderDebug extends CommandBase 
{
  DriveTrain driveTrain;
  double initialTicks;
  double ticksPerRev = 2048/10.71;
  double wheelDiam = 6; //This is in inches
  /** Creates a new EncoderDebug. */
  public EncoderDebug(DriveTrain driveTrain) 
  {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    initialTicks = driveTrain.getRightEncoderTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double currentTicks = driveTrain.getRightEncoderTicks() - initialTicks;
    double currentRevs = currentTicks / ticksPerRev;
    double distanceTraveled = currentRevs * wheelDiam * Math.PI;
    System.out.println("Distance Traveled: "  + distanceTraveled);
    SmartDashboard.putNumber("Encoder Distance", distanceTraveled);
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
