// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX rightLeader;
  private WPI_TalonFX rightFollower;
  private WPI_TalonFX leftLeader;
  private WPI_TalonFX leftFollower;

  Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
  Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);

  private AHRS gyro;

  private DifferentialDriveOdometry odometry;

  private DifferentialDrive differentialDrive;
  private DifferentialDriveKinematics differentialDriveKinematics;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    
    rightLeader = new WPI_TalonFX(RobotMap.DriveTrainMap.rightLeaderCanID);
    leftLeader = new WPI_TalonFX(RobotMap.DriveTrainMap.leftLeaderCanID);

    leftFollower = new WPI_TalonFX(RobotMap.DriveTrainMap.leftFollowerCanID);
    // leftFollower.follow(leftLeader);
    rightFollower = new WPI_TalonFX(RobotMap.DriveTrainMap.rightFollowerCanID);
    // rightFollower.follow(rightLeader);

    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Coast);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Coast);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setInverted(false);
    rightLeader.setInverted(false);

    MotorControllerGroup leftSide = new MotorControllerGroup(leftLeader, leftFollower);
    MotorControllerGroup rightSide = new MotorControllerGroup(rightLeader, rightFollower);

    gyro = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    differentialDrive = new DifferentialDrive(leftSide, rightSide);
    differentialDrive.setSafetyEnabled(true);

    rightEncoder.setReverseDirection(true);
    leftEncoder.setDistancePerPulse(RobotMap.DriveTrainMap.inchesPerPulse);
    rightEncoder.setDistancePerPulse(RobotMap.DriveTrainMap.inchesPerPulse);

    differentialDriveKinematics = new DifferentialDriveKinematics(RobotMap.DriveTrainMap.kTrackwidthMeters);
    resetEncoders();
    gyro.reset();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    odometry.update(
        gyro.getRotation2d(), 
        leftLeader.getSelectedSensorPosition() * RobotMap.DriveTrainMap.ticksToMeters, 
        rightLeader.getSelectedSensorPosition() * RobotMap.DriveTrainMap.ticksToMeters
        );

  }

  public double getVoltage(){
    return leftLeader.getMotorOutputVoltage();
  }

  public void tankDrive(double left, double right) {
    differentialDrive.tankDrive(left, right);   
 }

 public double getAngle(){
   return gyro.getAngle();
 }

 public double getLeftEncoderTicks(){
   return leftLeader.getSelectedSensorPosition();
 }

 public double getRightEncoderTicks(){
  return rightLeader.getSelectedSensorPosition();
 }

 public double getLeftFollowerEncoderTicks(){
  return leftFollower.getSelectedSensorPosition();
}

public double getRightFollowerEncoderTicks(){
 return rightFollower.getSelectedSensorPosition();
}

 public AHRS getGyro(){
   return gyro;
 }

 public Pose2d getPose2d(){
   return odometry.getPoseMeters();
 }

 public DifferentialDriveKinematics getKinematics(){
  return differentialDriveKinematics;
 }

 public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(
    leftEncoder.getRate(),
    rightEncoder.getRate()
    );

}

public void resetOdometry(Pose2d pose) {
  resetEncoders();
  odometry.resetPosition(pose, gyro.getRotation2d());
}

public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
}

public void tankDriveVolts(double leftVolts, double rightVolts) {
  leftLeader.setVoltage(leftVolts);
  rightLeader.setVoltage(rightVolts);
  differentialDrive.feed();
}

}
