// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import javax.management.openmbean.TabularType;

import com.kauailabs.navx.IMUProtocol.GyroUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain driveTrain;
  double targetAngle;
  private AHRS gyro;
  private double speed = 0.75;
  private boolean isDone;
  private double startingAngle = 0;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, double targetAngle) {
    this.driveTrain = driveTrain;
    this.targetAngle = targetAngle;
    gyro = driveTrain.getGyro();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // gyro.reset();
    // gyro.zeroYaw();
    startingAngle = gyro.getAngle();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleError = GetAngleTurned();
    if (angleError >= targetAngle + 7) { // counterclockwise
      driveTrain.tankDrive(-speed, speed);
    } else if (angleError <= targetAngle - 7) { // clockwise
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

  public double GetAngleTurned()
  {
    double angle = gyro.getAngle()-startingAngle;
    angle = angle%360;
    if(angle > 180){ angle-=360; }
    return angle;
  }
}
