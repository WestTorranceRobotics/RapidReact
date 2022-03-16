// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngleWithVisionTakeover extends CommandBase {
  private DriveTrain driveTrain;
  private PIDController anglePID;
  private AHRS gyro;

  private double targetAngle;
  private double initTurningSpeed = 0.75;
  private boolean ballFound;
  private boolean isAligned;

  private boolean isDone;
  private NetworkTable VTable = NetworkTableInstance.getDefault().getTable("Vision");

  public TurnToAngleWithVisionTakeover(DriveTrain driveTrain, double targetAngle) {
    this.driveTrain = driveTrain;
    anglePID = driveTrain.getAngleController();
    gyro = driveTrain.getGyro();

    this.targetAngle = targetAngle;
    ballFound = false;
    isAligned = false;
    isDone = false;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.reset();
    gyro.zeroYaw();

    anglePID.setSetpoint(0);
    anglePID.reset();
    driveTrain.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = NetworkTableInstance.getDefault().getTable("Vision").getEntry("vx").getDouble(-999);
    double vy = NetworkTableInstance.getDefault().getTable("Vision").getEntry("vy").getDouble(-999);
    // if (vx != -999 && vy != -999) {
    //   ballFound = true;
    // }

    // if (ballFound) {
      double leftCommand = 0;
      double rightCommand = 0;

      /* turn to face the target */ 
      // if (!isAligned) {
      double steeringAdjust = 0;
      anglePID.setP(VTable.getEntry("kP").getDouble(0.1945392));
      anglePID.setI(VTable.getEntry("kI").getDouble(0));
      steeringAdjust = MathUtil.clamp(anglePID.calculate(vx), -0.8, 0.8);
      
      leftCommand -= steeringAdjust;
      rightCommand += steeringAdjust;
      if (Math.abs(vx) <= 20) { // if the robot is facing within 20 pixels either side of the ball
        isAligned = true;
      }
      // }
      // else {

      /* moves towards the ball until the ball stops being seen, meaning the ball has been intaked (hopefully) */
      
      // double distAdjust = MathUtil.clamp(distancePID.calculate(vy), -0.5, 0.5);

      // leftCommand += distAdjust;
      // rightCommand += distAdjust;
    // }

      driveTrain.tankDrive(leftCommand, rightCommand);

    // }
    // else {
    //   // blindly turning to a general angle
    //   if (driveTrain.getAngle() >= targetAngle + 10) { // counterclockwise
    //     driveTrain.tankDrive(-initTurningSpeed, initTurningSpeed);
    //   } 
    //   else if (driveTrain.getAngle() <= targetAngle - 10) { // clockwise
    //     driveTrain.tankDrive(initTurningSpeed, -initTurningSpeed);
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
