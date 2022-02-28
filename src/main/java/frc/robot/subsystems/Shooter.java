/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot2020.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ShooterMap;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Shooter extends SubsystemBase {
  private CANSparkMax shootMotorFollower = new CANSparkMax(RobotMap.ShooterMap.ShooterFollowerCANID, MotorType.kBrushless);
  private CANSparkMax shootMotorLeader = new CANSparkMax(RobotMap.ShooterMap.ShooterLeaderCANID, MotorType.kBrushless);
  // private CANPIDController shootPID;
  private boolean atSpeed;
  private int ballsShot = 0;
  private boolean passedBallCurrent = false;
 
  public Shooter() {

//leader
    shootMotorLeader.restoreFactoryDefaults();
    shootMotorLeader.setIdleMode(IdleMode.kCoast);
    shootMotorLeader.setInverted(true);
    shootMotorLeader.getPIDController().setP(RobotMap.ShooterMap.kP);
    shootMotorLeader.getPIDController().setD(RobotMap.ShooterMap.kD);
    shootMotorLeader.getPIDController().setFF(0.8);
    shootMotorLeader.getPIDController().setOutputRange(-1, 1);
  //follower
    shootMotorFollower.restoreFactoryDefaults();
    shootMotorFollower.setIdleMode(IdleMode.kCoast);
    // shootMotorFollower.setInverted(true);
    shootMotorFollower.getPIDController().setP(RobotMap.ShooterMap.kP);
    shootMotorFollower.getPIDController().setD(RobotMap.ShooterMap.kD);
    shootMotorFollower.getPIDController().setFF(0.8);
    shootMotorFollower.getPIDController().setOutputRange(-1, 1);
    //shootMotorFollower.follow(shootMotorLeader, true);
  
  }

  public boolean active() {
    return shootMotorLeader.getAppliedOutput() != 0;
  }

  public double getCurrent() {
    return shootMotorLeader.getOutputCurrent();
  }

  public void currentWatch() {
    if (shootMotorLeader.getOutputCurrent() >= RobotMap.ShooterMap.ballCurrent && passedBallCurrent == false) {
      passedBallCurrent = true;
      ballsShot = ballsShot + 1;
    } else if (passedBallCurrent == true && shootMotorLeader.getOutputCurrent() < RobotMap.ShooterMap.ballCurrent-7) {
      passedBallCurrent = false;
    }
  }
  
  public double getVelocity() {
    return (shootMotorLeader.getEncoder().getVelocity() / RobotMap.ShooterMap.gearRatio); 
   }
   
   public void setVelocity(double velocity){
    shootMotorLeader.getPIDController().setReference(-velocity, com.revrobotics.CANSparkMax.ControlType.kVelocity);
    shootMotorFollower.getPIDController().setReference(-velocity, com.revrobotics.CANSparkMax.ControlType.kVelocity);
    
   }

   public double getVoltage() {
     return shootMotorLeader.getBusVoltage();
   }
  
   public void atSpeed(boolean atSpeed) {
    this.atSpeed= atSpeed;
  }
  public boolean atSpeed() {
    return this.atSpeed;
  }

  public void setPower(double power) {
    shootMotorLeader.set(-power);
    shootMotorFollower.set(-power);
  }

  public int getBallsShot() {
    return ballsShot;
  }

  /**
   * @deprecated Unreliable with higher loader speeds at the present
   */
  // public void currentWatch(double targetRPM) {
  //   if (shootMotorLeader.getOutputCurrent() >= RobotMap.ShooterMap.ballCurrent && passedBallCurrent == false) {
  //     passedBallCurrent = true;
  //     ballsShot = ballsShot + 1;
  //   } else if (passedBallCurrent == true && shootMotorLeader.getOutputCurrent() < RobotMap.ShooterMap.ballCurrent-7) {
  //     passedBallCurrent = false;
  //   }
  // }

  // public void directVolts(double volts) {
  //   shootMotorLeader.setVoltage(volts);
  // }

    // public void disablePID() {
  //   shootPID.setD(0);
  //   shootPID.setP(0);
  //   shootPID.setFF(0);
  // }

  // public void enablePID() {
  //   shootPID.setD(RobotMap.ShooterMap.Kd);
  //   shootPID.setP(RobotMap.ShooterMap.Kp);
  //   shootPID.setFF(RobotMap.ShooterMap.Kf);
  // }

  // public void updatePID() {
  //   shootPID.setD(SmartDashboard.getNumber("DSHOOT", RobotMap.ShooterMap.Kd));
  //   shootPID.setP(SmartDashboard.getNumber("PSHOOT", RobotMap.ShooterMap.Kp));
  //   shootPID.setFF(SmartDashboard.getNumber("FSHOOT", RobotMap.ShooterMap.Kf));
  // }

  /**
   * @param targetRPM desired RPM of shooter
   */
  // public void startShooter() {
  //   enablePID();
  //   shootPID.setReference(RobotMap.ShooterMap.lineShootRPM, ControlType.kVelocity);
  // }

  // public void startShooter(double rpm) {
  //   shootPID.setReference(rpm, ControlType.kVelocity);
  // }

  //   public void stopShooter () {
  //     disablePID();
  //    shootMotorLeader.set(0);
  //   }

}