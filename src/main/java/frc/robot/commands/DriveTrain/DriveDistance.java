// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase 
{
  DriveTrain driveTrain;
  double initialTicks;
  double ticksPerRev = 2048/10.71;
  double wierdnessFactor = 55.25/20.40473632391298 * 50.25/60.58816425120775;//This value is the experimental distance divided by the encoder distance
  double wheelDiam = 6; //This is in inches

  double speed = 0.6;
  double distanceToTravel;

  boolean isFinished = false;
  /** Creates a new DriveDistance. */
  public DriveDistance(DriveTrain driveTrain, double distanceToTravel) 
  {

    this.driveTrain = driveTrain;
    this.distanceToTravel = distanceToTravel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isFinished = false;
    System.out.println("Running Drive Distance");
    initialTicks = driveTrain.getRightEncoderTicks();
    
    if(distanceToTravel == 0)//Don't move
    {
      System.out.println("Disabling beacuse the distance to travel is 0.");
      isFinished = true;
    }
    else if(distanceToTravel > 0)//Move Forward
    {
      driveTrain.tankDrive(speed, speed);
    }
    else//Move Backwards
    {
      driveTrain.tankDrive(-speed, -speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double currentTicks = driveTrain.getRightEncoderTicks() - initialTicks;
    double currentRevs = currentTicks / ticksPerRev;
    double distanceTraveled = currentRevs * wheelDiam * Math.PI * wierdnessFactor;
    System.out.println("Distance Traveled: "  + distanceTraveled + "   Distance to Travel: " + distanceToTravel);
    SmartDashboard.putNumber("Encoder Distance", distanceTraveled);

    if(Math.abs(distanceTraveled) >= Math.abs(distanceToTravel))//If we are at or exceed the target distance
    {
      System.out.println("Disabling because we reached the target distance.");
      isFinished = true;
    }
    else if(distanceToTravel > 0)//Move Forward
    {
      driveTrain.tankDrive(speed, speed);
    }
    else//Move Backwards
    {
      driveTrain.tankDrive(-speed, -speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Ending");
    driveTrain.tankDrive(0, 0);//Stop Moving
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return isFinished;
  }
}
