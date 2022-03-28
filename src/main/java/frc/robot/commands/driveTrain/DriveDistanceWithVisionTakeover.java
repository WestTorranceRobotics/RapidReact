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


public class DriveDistanceWithVisionTakeover extends CommandBase {
  private DriveTrain driveTrain;
  private PIDController anglePID;
  private PIDController distancePID;
  private AHRS gyro;

  private boolean ballFound;
  private boolean isAligned;

  private boolean isDone;

  private NetworkTable VTable = NetworkTableInstance.getDefault().getTable("Vision");

  private double startingAngle;

  /** Creates a new DriveDistanceWithVisionTakeover. */
  public DriveDistanceWithVisionTakeover(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    anglePID = driveTrain.getAngleController();
    distancePID = driveTrain.getDistanceController();
    gyro = driveTrain.getGyro();

    ballFound = false;
    isAligned = false;
    isDone = false;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballFound = false;
    // gyro.reset();
    // gyro.zeroYaw();
    // startingAngle = gyro.getAngle();

    // anglePID.setSetpoint(0);
    // anglePID.reset();
    // driveTrain.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("driivng");
    double vx = NetworkTableInstance.getDefault().getTable("Vision").getEntry("vx").getDouble(-999);
    double vy = NetworkTableInstance.getDefault().getTable("Vision").getEntry("vy").getDouble(-999);
    boolean targetAcquired = NetworkTableInstance.getDefault().getTable("Vision").getEntry("Ball Found").getBoolean(false);
    if (vx != -999 && vy != -999) {
      ballFound = true;
    }

    if (ballFound) {
    // if (targetAcquired) {
      double leftCommand = 0;
      double rightCommand = 0;

      // /* turn to face the target  */
      // double steeringAdjust = 0;
      // anglePID.setP(0.01772);
      // anglePID.setI(0.011);
      
      // steeringAdjust = MathUtil.clamp(anglePID.calculate(vx), -1, 1);
      
      // leftCommand -= steeringAdjust;
      // rightCommand += steeringAdjust;
      /* moves towards the ball until the ball stops being seen, meaning the ball has been intaked (hopefully) 
      after the ball goes past a vy value, a timer will start. Once it finishes, the command will end. The next command
      will be drive distance with constant intaking. Then the shooter will aim and shoot. */

      driveTrain.tankDrive(0.7, 0.7);

      if (vy <= -20) {
        System.out.println("DONE");
        isDone = true;
      }
    }
    // else {
    //   // just drive 
    //   double speed = 0.7;
    //   driveTrain.tankDrive(speed, speed);
    // }
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
    if(angle > 180){ angle-=180; }
    return angle;
  }
}
