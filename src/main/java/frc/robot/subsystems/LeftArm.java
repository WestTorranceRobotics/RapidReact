// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.chrono.IsoChronology;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LeftArm extends SubsystemBase {
  private CANSparkMax leftArm;
  private boolean overrideEnabled;
  /** Creates a new LeftArm. */
  public LeftArm() {
    leftArm = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    leftArm.restoreFactoryDefaults();
    leftArm.setIdleMode(IdleMode.kBrake);
    leftArm.getEncoder().setPosition(0);
    overrideEnabled = false;
  }

  public void armForwards(){
    leftArm.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void armBackwards(){
    leftArm.set(-RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void stopArm(){
    leftArm.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  public double getEncoderPosition() {
    return leftArm.getEncoder().getPosition();
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
