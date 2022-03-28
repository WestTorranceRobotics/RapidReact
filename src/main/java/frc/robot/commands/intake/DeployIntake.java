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
  double deployedValue = 0;
  /** Creates a new DeployIntake. */
  public DeployIntake(Intake intake) {
    mIntake = intake;

    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mIntake.getDeployMotor().getEncoder().getPosition() <= RobotMap.IntakeMap.encoderValueForUndeployed) {
      isDeployed = false;
      //mIntake.getDeployMotor().getEncoder().getPosition();
    }
    else {
      isDeployed = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(isDeployed);
    if(!isDeployed && isFinished == false){
      mIntake.deployIntake();
     // mIntake.getAnalogIntakeValue() >= RobotMap.IntakeMap.voltageValueForDeployedLower && 
    if(mIntake.getDeployMotor().getEncoder().getPosition() >= RobotMap.IntakeMap.encoderValueForDeployed)
      {
        mIntake.stopIntake();
        isFinished = true;
      }
    }

    //mIntake.deployIntake();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.stopDeployMotors();
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
