// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import javax.sound.midi.MidiChannel;
import javax.sound.midi.SysexMessage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class UndeployIntake extends CommandBase {
  Intake mIntake;
  boolean isFinished = false;
  boolean isDeployed;
  /** Creates a new UndeployIntake. */
  public UndeployIntake(Intake intake) {
    mIntake = intake;

    addRequirements(mIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mIntake.getDeployMotor().getEncoder().getPosition() >= RobotMap.IntakeMap.encoderValueForUndeployed)
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
    if(isDeployed && isFinished == false){
      mIntake.unDeployIntake();
    if(mIntake.getDeployMotor().getEncoder().getPosition() <= 0)
      {
        mIntake.stopDeployMotors();
        isFinished = true;
      }
    }

    // mIntake.unDeployIntake();

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
