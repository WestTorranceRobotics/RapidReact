// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.IntakeMap;

public class Intake extends SubsystemBase 
{
  //Variables for running intake
  private CANSparkMax intakeMotor;

  //Variables for deploying intake
  private TalonSRX deployMotor;
  private Encoder deployEncoder;
  private boolean isDeployed;

  /** Creates a new Intake. */
  public Intake() 
  {
    intakeMotor = new CANSparkMax(RobotMap.IntakeMap.intakeMotorCANID, MotorType.kBrushless);
    intakeMotor.setInverted(true);

    deployMotor = new TalonSRX(RobotMap.IntakeMap.intakeDeployMotorCANID);
    deployMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void RunIntake()
  {
    intakeMotor.set(RobotMap.IntakeMap.intakeMotorPower);
  }

  public void ReverseIntake()
  {
    intakeMotor.set(RobotMap.IntakeMap.intakeMotorPower * -1);
  }

  public void StopIntake()
  {
    intakeMotor.set(0);
  }

  public TalonSRX getDeployMotor(){
    return deployMotor;
  }

  public void deployIntake(){
    deployMotor.set(ControlMode.PercentOutput, 0.55);
  }

  public void unDeployIntake(){
    deployMotor.set(ControlMode.PercentOutput, -0.55);
  }

  public void stopIntake(){
    deployMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isDeployed(){
    if(getAnalogIntakeValue() >= RobotMap.IntakeMap.voltageValueForDeployedLower && getAnalogIntakeValue() <= RobotMap.IntakeMap.voltageValueForDeployedUpper)
      {
        return true;
      }
    if(getAnalogIntakeValue() < RobotMap.IntakeMap.voltageValueForDeployedLower){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean ToggleIsDeployed()
  {
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
    // This method will be called once per scheduler run
  }
}
