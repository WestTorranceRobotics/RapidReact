// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickTankDrive extends CommandBase {
  private Joystick leftHand;
  private Joystick rightHand;
  private double leftHandIn;
  private double rightHandIn;
  private final DriveTrain driveTrain;

  /** Creates a new TankDrive. */
  public JoystickTankDrive(Joystick leftHand, Joystick rightHand, DriveTrain driveTrain) {
    this.leftHand = leftHand;
    this.rightHand = rightHand;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //   if (leftHand.getY() > 0.1 || rightHand.getY() > 0.1 || rightHand.getY() < -0.1 || leftHand.getY() < -0.1){
      
  // }
  leftHandIn = Math.pow(leftHand.getY(), 1);
  rightHandIn = Math.pow(rightHand.getY(), 1);
  driveTrain.tankDrive(leftHandIn, rightHandIn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
