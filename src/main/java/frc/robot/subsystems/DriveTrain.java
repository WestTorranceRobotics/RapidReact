// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX rightLeader;
  private WPI_TalonFX rightFollower;
  private WPI_TalonFX leftLeader;
  private WPI_TalonFX leftFollower;

  private DifferentialDrive differentialDrive;
  private PIDController controller;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private boolean isAutomatic = false;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightLeader = new WPI_TalonFX(RobotMap.DriveTrainMap.rightLeaderCanID);
    leftLeader = new WPI_TalonFX(RobotMap.DriveTrainMap.leftLeaderCanID);

    leftFollower = new WPI_TalonFX(RobotMap.DriveTrainMap.leftFollowerCanID);
    leftFollower.follow(leftLeader);
    rightFollower = new WPI_TalonFX(RobotMap.DriveTrainMap.rightFollowerCanID);
    rightFollower.follow(rightLeader);

    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Coast);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Coast);

    leftLeader.setInverted(true);
    rightLeader.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);
    differentialDrive.setSafetyEnabled(true);

    controller = new PIDController(kP, kI, kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double left, double right) {
    differentialDrive.tankDrive(left, -right);   
  }

  public void setAutomatic(boolean auto) {
    isAutomatic = auto;
  }
  
  public boolean isAutomatic() {
    return isAutomatic;
  }

  public PIDController getController() {
    return controller;
  }

  public void setP(double kP) {
    this.kP = kP;
    controller.setP(kP);
  }

  public double getP() {
    return kP;
  }

  public void setI(double kI) {
    this.kI = kI;
    controller.setP(kI);
  }

  public double getI() {
    return kI;
  }

  public void setD(double kD) {
    this.kD = kD;
    controller.setP(kD);
  }

  public double getD() {
    return kD;
  }

  public void enablePID() {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  public void disablePID() {
    controller.setP(0.0);
    controller.setI(0.0);
    controller.setD(0.0);
  }

  public void isAutomatic(boolean b) {
  }

}