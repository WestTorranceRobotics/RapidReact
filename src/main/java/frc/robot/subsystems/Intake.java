// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase 
{
  //Variables for running intake
  private CANSparkMax intakeMotor;

  //Variables for deploying intake
  private CANSparkMax deployMotor;
  private CANSparkMax deployMotorFollower;
  private Encoder deployEncoder;
  private boolean isDeployed;
  private DigitalInput limitSwitch;

  private PIDController controller;

  private double deploySpeed = 0.4;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(RobotMap.IntakeMap.intakeMotorCANID, MotorType.kBrushless);
    intakeMotor.setInverted(true);
    limitSwitch = new DigitalInput(4);

    deployMotor = new CANSparkMax(RobotMap.IntakeMap.intakeDeployMotorCANID, MotorType.kBrushless);
    deployMotor.setInverted(true);
    deployMotorFollower = new CANSparkMax(RobotMap.IntakeMap.intakeDeployFollowerCANID, MotorType.kBrushless);

    // deployMotorFollower.follow(deployMotor, true);
    deployMotorFollower.setInverted(false);

    deployMotor.setIdleMode(IdleMode.kBrake);
    deployMotorFollower.setIdleMode(IdleMode.kBrake);
    
    deployMotor.getEncoder().setPosition(0);
    deployMotorFollower.getEncoder().setPosition(0);

    // deployMotor.getPIDController().setP(RobotMap.IntakeMap.intakeDeployKp);
    // deployMotor.getPIDController().setI(RobotMap.IntakeMap.intakeDeployKi);
    // deployMotor.getPIDController().setD(RobotMap.IntakeMap.intakeDeployKd);
    // deployMotor.getPIDController().setOutputRange(-0.5, 0.5);
    
    //deployMotorFollower.setInverted(true);
    
    controller = new PIDController(0, 0, 0);
  }

  public boolean isActivated(){
    return limitSwitch.get();
    //The limit switched is pressed when limitswitch.get() is false.
  }

  public void runIntake() {
    intakeMotor.set(RobotMap.IntakeMap.intakeMotorPower);
  }

  public void reverseIntake() {
    intakeMotor.set(-RobotMap.IntakeMap.intakeMotorPower);
  }

  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  public void reverseIntake(double power) {
    intakeMotor.set(-power);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void deployIntake() {
    deployMotor.set(deploySpeed); // 0.75
    deployMotorFollower.set(deploySpeed);
  }

  public void unDeployIntake() {
    deployMotor.set(-deploySpeed); // -0.75
    deployMotorFollower.set(-deploySpeed);
  }

  public void stopDeployMotors() {
    deployMotor.set(0);
    deployMotorFollower.set(0);
  }

  public void setIntake(double angle){
    deployMotor.getPIDController().setReference(angle, ControlType.kPosition);
  }

  public double getIntake() {
    return deployMotor.getEncoder().getPosition();
  }

  public CANSparkMax getDeployMotor() {
    return deployMotor;
  }

  public CANSparkMax getFollowerMotor() {
    return deployMotorFollower;
  }

  public PIDController getController() {
    return controller;
  }

  public boolean isDeployed() {
    if (getDeployMotor().getEncoder().getPosition() >= RobotMap.IntakeMap.encoderValueForUndeployed)
      {
        return true;
      }
    // if (getDeployMotor().getEncoder().getPosition() <= RobotMap.IntakeMap.encoderValueForUndeployed) {
    //   return true;
    // }
    else {
      return false;
    }
  }

  public boolean isRunning() {
    return intakeMotor.getOutputCurrent() != 0;
  }

  public boolean toggleIsDeployed() {
    isDeployed = !isDeployed;//toggles the value of is deployed
    return !isDeployed;//returns the initial value
  }

  public double getAnalogIntakeValue() {
    AnalogPotentiometer potentiometerInput = new AnalogPotentiometer(0);

    double voltage = potentiometerInput.get();

    potentiometerInput.close();

    return voltage;
  }


  @Override
  public void periodic() 
  {
    // double error = getAnalogIntakeValue() - setpoint;
    // double kP = 0.02; 
    // double power = kP * error;
  }
}
