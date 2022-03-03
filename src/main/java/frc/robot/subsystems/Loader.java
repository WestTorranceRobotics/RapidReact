// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Loader extends SubsystemBase {

  CANSparkMax loaderMotor;
  /** Creates a new Loader. */
  public Loader() {
    loaderMotor = new CANSparkMax(RobotMap.LoaderMap.loaderMotorCANID, MotorType.kBrushless);
    loaderMotor.restoreFactoryDefaults();

  }

  public void runLoader(){
    loaderMotor.set(-0.4);
  }

  public void reverseLoader(){
    loaderMotor.set(0.5);
  }

  public void stopLoader(){
    loaderMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public double getAppliedOutput() {
    return loaderMotor.getAppliedOutput();
}
}
