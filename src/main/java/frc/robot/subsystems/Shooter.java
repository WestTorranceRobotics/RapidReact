/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Shooter extends SubsystemBase {
  private CANSparkMax shootMotorFollower = new CANSparkMax(RobotMap.ShooterMap.shootFollowerCanID, MotorType.kBrushless);
  private CANSparkMax shootMotorLeader = new CANSparkMax(RobotMap.ShooterMap.shootLeaderCanID, MotorType.kBrushless);
  private CANPIDController shootPID;
  private boolean atSpeed;
  private int ballsShot = 0;
  private boolean passedBallCurrent = false;
  private boolean automatic;
 
  public Shooter() {
    shootMotorFollower.restoreFactoryDefaults();
    shootMotorLeader.restoreFactoryDefaults();
    shootMotorFollower.follow(shootMotorLeader, true);
    shootMotorLeader.setIdleMode(IdleMode.kCoast);
    shootMotorFollower.setIdleMode(IdleMode.kCoast);
    shootMotorLeader.setInverted(true);
    shootMotorFollower.setInverted(true);
    // shootPID = shootMotorLeader.getPIDController();
    // shootPID.setD(RobotMap.ShooterMap.Kd);
    // shootPID.setP(RobotMap.ShooterMap.Kp);
    // shootPID.setFF(RobotMap.ShooterMap.Kf);
    // shootPID.setReference(0, ControlType.kVelocity);
  }

  public boolean active() {
    return shootMotorLeader.getAppliedOutput() != 0;
  }

  public int getBallsShot() {
    return ballsShot;
  }

  public double getCurrent() {
    return shootMotorLeader.getOutputCurrent();
  }

  public void resetBallsShot() {
    ballsShot = 0;
  }

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
  public void startShooter() {
   // enablePID();
    shootPID.setReference(RobotMap.ShooterMap.lineShootRPM, ControlType.kVelocity);
  }

  public void startShooter(double rpm) {
    shootPID.setReference(rpm, ControlType.kVelocity);
  }

    public void stopShooter () {
     // disablePID();
     shootMotorLeader.set(0);
    }
  
  public double getVelocity() {
    return (shootMotorLeader.getEncoder().getVelocity() / RobotMap.ShooterMap.gearRatio); 
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

  public void directPower (double power) {
    shootMotorLeader.set(power);
  }
 
  /**
   * @deprecated Unreliable with higher loader speeds at the present
   */
  /*
  public void currentWatch(double targetRPM) {
    if (shootMotorLeader.getOutputCurrent() >= RobotMap.ShooterMap.ballCurrent && passedBallCurrent == false) {
      passedBallCurrent = true;
      ballsShot = ballsShot + 1;
    } else if (passedBallCurrent == true && shootMotorLeader.getOutputCurrent() < RobotMap.ShooterMap.ballCurrent-7) {
      passedBallCurrent = false;
    }
  }
*/
  public void directVolts(double volts) {
    shootMotorLeader.setVoltage(volts);
  }

  @Override
  public void periodic() {
    // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // double angle = ty + RobotMap.limelightAngle;
    // double tan = Math.tan(Math.toRadians(angle));
    // double dx = (RobotMap.targetHeight - RobotMap.limelightHeight) / tan;
  }
}