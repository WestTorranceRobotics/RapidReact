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

public class TurnToDirectionTest extends CommandBase {
  private DriveTrain driveTrain;
  private double targetDirection;
  private AHRS gyro;
  private double speed = 0.55;
  private boolean isDone = false;
  private PIDController pidController = new PIDController(0, 0, 0);

  /** Creates a new TurnToAngle. */
  public TurnToDirectionTest(DriveTrain driveTrain, double targetDirection) {
    this.driveTrain = driveTrain;
    this.targetDirection = targetDirection;
    gyro = driveTrain.getGyro();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    pidController.setP(0.01);
    pidController.setI(0.01);
    pidController.setD(0.00153242);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleError = GetAngleError();

    double spd = MathUtil.clamp(pidController.calculate(angleError), -.85, .85);
    System.out.println("PID Speed is " + spd + " Error is " + angleError);

    driveTrain.tankDrive(spd, -spd);

    // System.out.println("Updating");
    // driveTrain.tankDrive(0.6, -0.6);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = false;
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
    if(angle > 180){ angle-=360; }
    return angle;
  }
}
