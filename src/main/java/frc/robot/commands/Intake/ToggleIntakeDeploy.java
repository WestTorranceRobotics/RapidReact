// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class ToggleIntakeDeploy extends CommandBase 
{
  private Intake intake;
  private Encoder deployEncoder;
  private TalonSRX deployMotor;
  private boolean isDeployed;
  private double currentPotentiometerVoltage;
  private double deployMotorPower;
  
  private double neededDeployTicks = 90*(1/360)*(4096/1);

  private boolean isFinished = false;
  /** Creates a new ToggleIntakeDeploy. */
  public ToggleIntakeDeploy(Intake _intake) 
  {
    intake = _intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isDeployed = intake.ToggleIsDeployed();
    deployMotor = intake.getDeployMotor();
    currentPotentiometerVoltage = intake.getAnalogIntakeValue();

    deployMotorPower = RobotMap.IntakeMap.deployMotorPower;

    if(isDeployed) //If the intake is deployed, retract intake
    {
      deployMotorPower *= -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    deployMotor.set(ControlMode.PercentOutput, deployMotorPower);

    if(isDeployed){
      if(Math.abs(currentPotentiometerVoltage - intake.getAnalogIntakeValue()) >= RobotMap.IntakeMap.voltageValueForDeployed)
      {
        isFinished = true;
      }
    }
    if(!isDeployed){
      if(Math.abs(currentPotentiometerVoltage - intake.getAnalogIntakeValue()) >= RobotMap.IntakeMap.voltageValueForUndeployed)
        {
          isFinished = true;
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    deployMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
