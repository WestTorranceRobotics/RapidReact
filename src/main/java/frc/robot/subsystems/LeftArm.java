// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LeftArm extends SubsystemBase {
  private CANSparkMax leftArm;
  /** Creates a new LeftArm. */
  public LeftArm() {
    leftArm = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    leftArm.restoreFactoryDefaults();
    leftArm.setIdleMode(IdleMode.kBrake);

  }

  public void liftForwards(){
    leftArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void liftBackwards(){
    leftArm.set(-RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void setNoPower(){
    leftArm.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
