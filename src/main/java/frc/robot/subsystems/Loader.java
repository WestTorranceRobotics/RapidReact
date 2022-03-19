// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Loader extends SubsystemBase {
  CANSparkMax loaderMotor;
  private boolean proxSensorEnabled = true;
  private AnalogInput ballDetector;
  /** Creates a new Loader. */
  public Loader() {
    loaderMotor = new CANSparkMax(RobotMap.LoaderMap.loaderMotorCANID, MotorType.kBrushless);
    loaderMotor.restoreFactoryDefaults();
  
    ballDetector = new AnalogInput(1);
  }

  public void runLoader(double power){
    loaderMotor.set(power); // -0.3
  }

  public double getProxVoltage() {
    return ballDetector.getVoltage();
  }

  public boolean seeBall(){
    if (proxSensorEnabled && ballDetector.getVoltage() < 1.5){
      return true;
    }
    return false;
  }

  public void reverseLoader() {
    loaderMotor.set(0.5);
  }

  public void stopLoader() {
    loaderMotor.set(0);
  }

  public boolean isRunning() {
    return Math.abs(loaderMotor.getAppliedOutput()) > 0;
  }

  public void enableProxSensor() {
    proxSensorEnabled = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAppliedOutput() {
      return loaderMotor.getAppliedOutput();
  }
}
