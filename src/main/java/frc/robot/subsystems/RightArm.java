// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class RightArm extends SubsystemBase {
  private CANSparkMax rightArm;
  /** Creates a new LeftArm. */
  public RightArm() {
    rightArm = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    rightArm.restoreFactoryDefaults();
    rightArm.setIdleMode(IdleMode.kBrake);

  }

  public void liftForwards(){
    rightArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void liftBackwards(){
    rightArm.set(-RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void setNoPower(){
    rightArm.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
