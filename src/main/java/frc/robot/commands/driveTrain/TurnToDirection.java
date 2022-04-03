// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import javax.management.openmbean.TabularType;

import com.kauailabs.navx.IMUProtocol.GyroUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToDirection extends CommandBase {
  DriveTrain driveTrain;
  double targetDirection;
  private PIDController anglePID;
  private AHRS gyro;
  private double speed = 0.7;
  private boolean isDone;

  /** Creates a new TurnToAngle. */
  public TurnToDirection(DriveTrain driveTrain, double targetDirection) {
    this.driveTrain = driveTrain;
    this.targetDirection = targetDirection;
    anglePID = driveTrain.getAngleController();
    gyro = driveTrain.getGyro();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.enablePID();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCommand = 0;
    double rightCommand = 0;
    double angleError = GetAngleError();
    
    anglePID.setP(0.021072);
    anglePID.setI(0.020);

    double steeringAdjust = 0;
    steeringAdjust = MathUtil.clamp(anglePID.calculate(angleError, 0), -0.8, 0.8);
    
    leftCommand -= steeringAdjust;
    rightCommand += steeringAdjust;
    driveTrain.tankDrive(leftCommand, rightCommand);
    System.out.println(angleError);
    // if (angleError >= 1) { // counterclockwise
    //   driveTrain.tankDrive(-speed, speed);
    // } else if (angleError <= -1) { // clockwise
    //   driveTrain.tankDrive(speed, -speed);
    // } else {
    //   isDone = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
    isDone = false;
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
    if(angle > 180){ angle-=360; }
    return angle;
  }
}
