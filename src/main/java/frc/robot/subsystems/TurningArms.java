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
  private CANSparkMax elevatorTurningLeader;
  private CANSparkMax elevatorTurningFollower;
  
  public TurningArms() {
    elevatorTurningLeader = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    elevatorTurningFollower = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningFollower, MotorType.kBrushless);
    elevatorTurningFollower.setIdleMode(IdleMode.kBrake);
    elevatorTurningLeader.setIdleMode(IdleMode.kBrake);
    elevatorTurningFollower.follow(elevatorTurningLeader);
    elevatorTurningLeader.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

  }

  public CANSparkMax getElevatorTurningLeader(){
    return elevatorTurningLeader;
  }

  public double getElevatorMotorTicks(){
    return elevatorTurningLeader.getEncoder().getPosition();
  }

  public void liftForwards(){
    elevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorMotorUp);
    elevatorTurningFollower.set(-RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void liftBackwards(){
    elevatorTurningLeader.set(-RobotMap.ElevatorMap.elevatorMotorDown);
    elevatorTurningFollower.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void setNoPower(){
    elevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorHalt);
    elevatorTurningFollower.set(RobotMap.ElevatorMap.elevatorHalt);
  }
}
