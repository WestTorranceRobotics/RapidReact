// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class DeployIntake extends CommandBase {
  Intake mIntake;
  boolean isDeployed;
  boolean isFinished = false;
  /** Creates a new DeployIntake. */
  public DeployIntake(Intake intake) {
    mIntake = intake;

    addRequirements(mIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("HIIIII");
    if(mIntake.getAnalogIntakeValue() >= RobotMap.IntakeMap.voltageValueForDeployedLower && mIntake.getAnalogIntakeValue() <= RobotMap.IntakeMap.voltageValueForDeployedUpper)
      {
        isDeployed = true;
      }
    else{
      isDeployed = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(isDeployed);
    if(!isDeployed && isFinished == false){
      mIntake.deployIntake();
     // mIntake.getAnalogIntakeValue() >= RobotMap.IntakeMap.voltageValueForDeployedLower && 
    if(mIntake.getAnalogIntakeValue() <= RobotMap.IntakeMap.voltageValueForDeployedUpper)
      {
        mIntake.stopIntake();
        isFinished = true;
      }
    }

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.getDeployMotor().set(ControlMode.PercentOutput, 0);
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
