// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  DriveTrain driveTrain;

  double speed;
  double distanceToTravel;
  double ticksToTravel;

  boolean isFinished = false;
  /** Drives in one direction at a speed for a distance (in inches). */
  public DriveDistance(DriveTrain driveTrain, double distanceToTravel, double speed) {
    this.driveTrain = driveTrain;
    this.distanceToTravel = distanceToTravel;
    this.speed = speed;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    driveTrain.getLeftEncoder().reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceTraveled = driveTrain.getLeftEncoder().getDistance();
    SmartDashboard.putNumber("Encoder Distance Traveled", distanceTraveled);

    if (Math.abs(distanceTraveled) >= Math.abs(distanceToTravel)) {
      isFinished = true;
    }
    else if (distanceToTravel > 0) {
      driveTrain.tankDrive(speed, speed);
    }
    else {
      driveTrain.tankDrive(-speed, -speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
