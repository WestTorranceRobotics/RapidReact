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
  private boolean overrideEnabled;
  /** Creates a new LeftArm. */
  public RightArm() {
    rightArm = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningFollower, MotorType.kBrushless);
    rightArm.restoreFactoryDefaults();
    rightArm.setIdleMode(IdleMode.kBrake);
    rightArm.getEncoder().setPosition(0);
    overrideEnabled = false;
  }

  public void armForwards(){
    rightArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void armBackwards(){
    rightArm.set(-RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void stopArm(){
    rightArm.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  public double getEncoderPosition() {
    return rightArm.getEncoder().getPosition();
  }

  public void toggleManualOverride() {
    overrideEnabled = !overrideEnabled;
  }

  public boolean isOverridden() {
    return overrideEnabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
