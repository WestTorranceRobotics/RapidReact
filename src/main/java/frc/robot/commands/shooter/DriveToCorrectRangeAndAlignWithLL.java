// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToCorrectRangeAndAlignWithLL extends CommandBase {
  private DriveTrain driveTrain;

  private PIDController anglePID;
  private double kP = 0.1345392;
  private double kI = 0;
  private double kD = 0.00503242;

  private PIDController distancePID;
  private double kPDist = 0.23;
  private double kIDist = 0.09;
  private double kDDist = 0;

  private double initTY = Double.MIN_VALUE;
  private boolean isAligned = false;
  private boolean isCorrectDistance = false;

  private NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("LLPID");

  /** Creates a new TurnToAngleUsingLimelight. */
  public DriveToCorrectRangeAndAlignWithLL(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    anglePID = driveTrain.getAngleController();
    distancePID = driveTrain.getDistanceController();
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.setP(kP);
    anglePID.setI(kI);
    anglePID.setD(kD);
    // driveTrain.enablePID();

    // turns on limelight and also lets rpi know that limelight is on
    NetworkTableInstance.getDefault().getTable("rpi").getEntry("aimbot").setDouble(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0.0);

    anglePID.setSetpoint(0);
    
    driveTrain.setAutomatic(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCommand = 0;
    double rightCommand = 0;

    /* stays on target distancewise
        -Only starts this section up when a valid target is found
            (could not put this simply in initialization because the limelight takes a while to start up)
        -Once a target has been found, it will check if the initialY has been initialized to a number that isn't the minimum value.
        If it has not, then it will set initY to the current ty that the robot is at.
        If it has initialized, then it will run PID to stay on target.*/
    boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    if (hasTarget) {
      if (initTY == Double.MIN_VALUE) {
        initTY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        // driveTrain.enablePID();
        distancePID.setSetpoint(initTY);
      } else {
        // maybe incorporate tx aligning in here as well
        System.out.println(initTY);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        distancePID.setP(LLTable.getEntry("distkP").getDouble(0));
        distancePID.setD(LLTable.getEntry("distkD").getDouble(0));

        double distancePowerLimit = 0;
        if (isAligned) {
          distancePowerLimit = 0.8;
        }

        double distAdjust = MathUtil.clamp(distancePID.calculate(ty, initTY), -distancePowerLimit, distancePowerLimit);
        LLTable.getEntry("distAdjust").setDouble(distAdjust);
        
        leftCommand -= distAdjust;
        rightCommand -= distAdjust;
      }
    }
    /* using the equation for distance and ty found in the limelight docs, we can 
    show a boolean on the shuffleboard that says "In range" for the drivers. This is
    so the drivers know that they can shoot. */
    

    /* turn to face the target */ 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    double steeringAdjust = 0;
    // anglePID.setP(LLTable.getEntry("anglekP").getDouble(0));
    // anglePID.setD(LLTable.getEntry("anglekD").getDouble(0));
    // checks for a threshold of accepted values
    if (Math.abs(tx) > 2.8) {
      steeringAdjust = MathUtil.clamp(anglePID.calculate(tx), -0.7, 0.7);
      LLTable.getEntry("steeringAdjust").setDouble(steeringAdjust);
      isAligned = false;
    } else {
      isAligned = true;
    }
    
    leftCommand -= steeringAdjust;
    rightCommand += steeringAdjust;
    driveTrain.tankDrive(leftCommand, rightCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setAutomatic(false);
    driveTrain.disablePID();
    NetworkTableInstance.getDefault().getTable("rpi").getEntry("aimbot").setDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}