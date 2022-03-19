// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class NewShooter extends SubsystemBase {
  private TalonFX topShooter = new TalonFX(RobotMap.ShooterMap.ShooterLeaderCANID);
  private TalonFX bottomShooter = new TalonFX(RobotMap.ShooterMap.ShooterFollowerCANID);
  /** Creates a new NewShooter. */
  public NewShooter() {
    topShooter.setNeutralMode(NeutralMode.Coast);
    bottomShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    topShooter.set(ControlMode.PercentOutput, -power);
    bottomShooter.set(ControlMode.PercentOutput, -power);
  }

  public void stopPower(){
    topShooter.set(ControlMode.PercentOutput, 0);
    bottomShooter.set(ControlMode.PercentOutput, 0);
  }
}
