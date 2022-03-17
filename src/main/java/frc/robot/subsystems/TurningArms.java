// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurningArms extends SubsystemBase {
  /** Creates a new TurningArms. */
  private CANSparkMax rightTurningArm;
  private CANSparkMax leftTurningArm;
  
  public TurningArms() {
    rightTurningArm = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    leftTurningArm = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningFollower, MotorType.kBrushless);
    leftTurningArm.setIdleMode(IdleMode.kBrake);
    rightTurningArm.setIdleMode(IdleMode.kBrake);
    leftTurningArm.setInverted(true);
    rightTurningArm.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

  }

  public double getElevatorMotorTicks(){
    return rightTurningArm.getEncoder().getPosition();
  }

  public void liftForwards(){
    rightTurningArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
    leftTurningArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void liftBackwards(){
    rightTurningArm.set(RobotMap.ElevatorMap.elevatorMotorDown);
    leftTurningArm.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void rightArmForwards() {
    rightTurningArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void rightArmBackwards() {
    rightTurningArm.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void leftArmForwards() {
    leftTurningArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void leftArmBackwards() {
    leftTurningArm.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void setNoPower(){
    rightTurningArm.set(RobotMap.ElevatorMap.elevatorHalt);
    leftTurningArm.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  public void stopLeftArm() {
    leftTurningArm.set(0);
  }

  public void stopRightArm() {
    rightTurningArm.set(0);
  }
}
