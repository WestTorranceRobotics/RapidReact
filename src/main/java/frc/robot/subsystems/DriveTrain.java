// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
  public static final String getRightEncoderTicks = null;
  private WPI_TalonSRX leftLeader;
  private WPI_TalonSRX rightLeader;
  private WPI_TalonSRX leftFollower;
  private WPI_TalonSRX rightFollower;

  Encoder leftEncoder = new Encoder(0,1, false, EncodingType.k4X);
  Encoder rightEncoder = new Encoder(2,3, false, EncodingType.k4X);

  private AHRS gyro;

  private DifferentialDrive differentialDrive;

  private PIDController anglePID;
  private PIDController distancePID;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private boolean isAutomatic = false;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    leftLeader = new WPI_TalonSRX(RobotMap.DriveTrainMap.leftLeaderCANID);
    leftFollower = new WPI_TalonSRX(RobotMap.DriveTrainMap.leftFollowerCANID);
    rightLeader = new WPI_TalonSRX(RobotMap.DriveTrainMap.rightLeaderCANID);
    rightFollower = new WPI_TalonSRX(RobotMap.DriveTrainMap.rightFollowerCANID);

    gyro = new AHRS(SPI.Port.kMXP);

    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    leftLeader.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightLeader.setInverted(true);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);   
    
    gyro.reset();
    gyro.zeroYaw();

    rightEncoder.setReverseDirection(true);
    leftEncoder.setDistancePerPulse(RobotMap.DriveTrainMap.inchesPerPulse);
    rightEncoder.setDistancePerPulse(RobotMap.DriveTrainMap.inchesPerPulse);

    anglePID = new PIDController(kP, kI, kD);
    distancePID = new PIDController(0, 0, 0);
  }

  public void tankDrive(double leftPower, double rightPower){
    differentialDrive.tankDrive(leftPower, rightPower);
    // leftLeader.set(ControlMode.PercentOutput, leftPower);
    // rightLeader.set(ControlMode.PercentOutput, rightPower);
  }

  public WPI_TalonSRX getleftLeader(){
    return leftLeader;
  }

  public double getLeftLeaderEncoder(){
    return leftLeader.getSelectedSensorPosition();
  }

  public AHRS getGyro(){
    return gyro;
  } 

  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  public double getRightDistance(){
    return rightEncoder.getDistance();
  }

  public double getAngle() {
    return gyro.getAngle();
  }
  public double getYaw() {
    return gyro.getYaw();
  }

  public double getLeftEncoderTicks(){
    return leftEncoder.get();
  }

  public Encoder getLeftEncoder(){
    return leftEncoder;
  }

  public TalonSRX getRightFollower(){
    return rightFollower;
  }

  public TalonSRX getRightLeader(){
    return rightLeader;
  }

  public void resetEncoder(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getRightEncoderTicks(){
    return rightEncoder.get();
  }

  public Encoder getRightEncoder(){
    return rightEncoder;
  }

  public void setAutomatic(boolean auto) {
    isAutomatic = auto;
  }
  
  public boolean isAutomatic() {
    return isAutomatic;
  }

  public PIDController getAngleController() {
    return anglePID;
  }

  public PIDController getDistanceController() {
    return distancePID;
  }

  public void setP(double kP) {
    this.kP = kP;
    anglePID.setP(kP);
  }

  public double getP() {
    return kP;
  }

  public void setI(double kI) {
    this.kI = kI;
    anglePID.setP(kI);
  }

  public double getI() {
    return kI;
  }

  public void setD(double kD) {
    this.kD = kD;
    anglePID.setP(kD);
  }

  public double getD() {
    return kD;
  }

  public void enablePID() {
    anglePID.setP(RobotMap.DriveTrainMap.kP);
    anglePID.setI(RobotMap.DriveTrainMap.kI);
    anglePID.setD(RobotMap.DriveTrainMap.kD);
    distancePID.setP(0.0);
    distancePID.setI(0.0);
    distancePID.setD(0.0);
  }

  public void disablePID() {
    anglePID.setP(0.0);
    anglePID.setI(0.0);
    anglePID.setD(0.0);
    distancePID.setP(0.0);
    distancePID.setI(0.0);
    distancePID.setD(0.0);
  }

  public double getDistanceFromTarget() {
    // hub is 8' 8" tall
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double radiansTY = ty * (3.14159 / 180.0);
    double h2 = 37.5;
    double angleLL = 50 * (3.14159 / 180.0);
    return (104 - h2) / Math.tan(angleLL + radiansTY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
