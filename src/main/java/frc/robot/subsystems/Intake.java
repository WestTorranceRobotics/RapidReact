// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.IntakeMap;

public class Intake extends SubsystemBase 
{
  //Variables for running intake
  private TalonSRX intakeMotor;

  //Variables for deploying intake
  private TalonSRX deployMotor;
  private Encoder deployEncoder;
  private boolean isDeployed;

  /** Creates a new Intake. */
  public Intake() 
  {
    intakeMotor = new TalonSRX(RobotMap.IntakeMap.intakeMotorCANID);
    intakeMotor.setInverted(true);

    deployMotor = new TalonSRX(RobotMap.IntakeMap.intakeDeployMotorCANID);
    deployEncoder = new Encoder(RobotMap.IntakeMap.deployEncoderChannel1,RobotMap.IntakeMap.deployEncoderChannel2);
    deployEncoder.reset();
  }

  public void RunIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, RobotMap.IntakeMap.intakeMotorPower);
  }

  public void ReverseIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, RobotMap.IntakeMap.intakeMotorPower * -1);
  }

  public void StopIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public Encoder getDeployEncoder(){return deployEncoder;}
  public TalonSRX getDeployMotor(){return deployMotor;}

  public boolean ToggleIsDeployed()
  {
    isDeployed = !isDeployed;//toggles the value of is deployed
    return !isDeployed;//returns the initial value
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
