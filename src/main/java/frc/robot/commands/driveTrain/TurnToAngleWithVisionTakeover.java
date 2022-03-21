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
  private PIDController distancePID;
  private AHRS gyro;

  private double targetAngle;
  private double initTurningSpeed = 0.69;
  private boolean ballFound;
  private boolean isAligned;

  private boolean isDone;
  private NetworkTable VTable = NetworkTableInstance.getDefault().getTable("Vision");

  private double startingAngle;

  public TurnToAngleWithVisionTakeover(DriveTrain driveTrain, double targetAngle) {
    this.driveTrain = driveTrain;
    anglePID = driveTrain.getAngleController();
    distancePID = driveTrain.getDistanceController();
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
    // gyro.reset();
    // gyro.zeroYaw();
    startingAngle = gyro.getAngle();

    anglePID.setSetpoint(0);
    anglePID.reset();
    driveTrain.enablePID();
    ballFound = false;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = NetworkTableInstance.getDefault().getTable("Vision").getEntry("vx").getDouble(-999);
    double vy = NetworkTableInstance.getDefault().getTable("Vision").getEntry("vy").getDouble(-999);
    boolean targetAcquired = NetworkTableInstance.getDefault().getTable("Vision").getEntry("Ball Found").getBoolean(false);
    if (vx != -999 && vy != -999) {
      ballFound = true;
    }

    if (ballFound) {
      double leftCommand = 0;
      double rightCommand = 0;

      /* turn to face the target  */
      double steeringAdjust = 0;
      anglePID.setP(0.01472);
      anglePID.setI(0.011);
      // anglePID.setP(VTable.getEntry("kP").getDouble(0.1945392));
      // anglePID.setI(VTable.getEntry("kI").getDouble(0));
      steeringAdjust = MathUtil.clamp(anglePID.calculate(vx), -1, 1);
      System.out.println(steeringAdjust);
      
      leftCommand -= steeringAdjust;
      rightCommand += steeringAdjust;
      /* moves towards the ball until the ball stops being seen, meaning the ball has been intaked (hopefully) 
      after the ball goes past a vy value, a timer will start. Once it finishes, the command will end. The next command
      will be drive distance with constant intaking. Then the shooter will aim and shoot. */
      double distAdjust = 0.7;

      leftCommand += distAdjust;
      rightCommand += distAdjust;

      driveTrain.tankDrive(leftCommand, rightCommand);

      if (vy <= -25) {
        isDone = true;
      }
    }
    else {
      // blindly turning to a general angle
      double angleTurned = GetAngleTurned();

      if (angleTurned >= targetAngle + 10) { // counterclockwise
        driveTrain.tankDrive(-initTurningSpeed, initTurningSpeed);
      } 
      else if (angleTurned <= targetAngle - 10) { // clockwise
        driveTrain.tankDrive(initTurningSpeed, -initTurningSpeed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.disablePID();
    isDone = false;
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
/*
4 ball auto:
-normal two ball auto (but with aiming)
-turn to angle towards human player station
-visionAssistedDriving:
  drive forward with no assistance
  When a ball is seen, use same vision takeover from turnToAngleWithVisionTakeover
  Basically, the algorithm is the same except that in the beginning, drive distance is used instead of turn to angle
*/
