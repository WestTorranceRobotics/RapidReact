// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import javax.management.openmbean.TabularType;

import com.kauailabs.navx.IMUProtocol.GyroUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToDirection extends CommandBase {
  DriveTrain driveTrain;
  double targetDirection;
  private AHRS gyro;
  private double speed = 0.75;
  private boolean isDone;

  /** Creates a new TurnToAngle. */
  public TurnToDirection(DriveTrain driveTrain, double targetDirection) {
    this.driveTrain = driveTrain;
    this.targetDirection = targetDirection;
    gyro = driveTrain.getGyro();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleError = GetAngleError();
    if (angleError >= 5) { // counterclockwise
      driveTrain.tankDrive(-speed, speed);
    } else if (angleError <= 5) { // clockwise
      driveTrain.tankDrive(speed, -speed);
    } else {
      isDone = true;
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
    return isDone;
  }

  public double GetAngleError()
  {
    double angle = gyro.getAngle()-targetDirection;
    angle = angle%360;
    if(angle > 180){ angle-=180; }
    return angle;
  }
}
