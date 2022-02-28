// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  /*
  * The AHRS class enables access to basic connectivity and state information, 
  * as well as key 6-axis and 9-axis orientation information (yaw, pitch, roll, 
  * compass heading, fused (9-axis) heading and magnetic disturbance detection.
  */

  Encoder leftEncoder = new Encoder(0,1, false, EncodingType.k4X);
  Encoder rightEncoder = new Encoder(2,3, false, EncodingType.k4X);

  private AHRS gyro;

  /*A class for driving differential drive/skid-steer drive platforms such as the Kit of Parts 
  drive base, "tank drive", or West Coast Drive.*/
  private DifferentialDrive differentialDrive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    /*Instantiate your motors or create an instance or object of the type of the WPI_TalonFX class.
      Basically, before the variables leftLeader, leftFollower, etc had no object that defined the
      variable and this is where you create an object of the same time. 

      Similar to how you assign a value to int x by writing x = 5;

      In the parameters of the constructors, you have the CANID value you assign to each of the motor
      controllers which allow our Roborio to be able to use them and control them. You get it by 
      accessing the RobotMap.DriveTrainMap.

      WPI_TalonFX are used to control our motors and be able to send power to our motors. We can also use
      motor controllors to get information from our motors like encoder ticks or the RPM the motor is spinning
      at. 
    */

    leftLeader = new WPI_TalonSRX(RobotMap.DriveTrainMap.leftLeaderCANID);
    leftFollower = new WPI_TalonSRX(RobotMap.DriveTrainMap.leftFollowerCANID);
    rightLeader = new WPI_TalonSRX(RobotMap.DriveTrainMap.rightLeaderCANID);
    rightFollower = new WPI_TalonSRX(RobotMap.DriveTrainMap.rightFollowerCANID);

    /*Instantiate your gyro or create an instance or object of the type of the AHRS class.
      Gyros are used to figure out the orientation/angle the robot is currently facing based on the when the robot 
      is initally reset. We can use this angle/orientation to manuever our robot and allow our robot to travel at 
      angles and we can get this angle by using the getPitch() method.
    */

    gyro = new AHRS(SPI.Port.kMXP);

    /*
      rightFollower and leftFollower are set to follower the leaders in this part of the code here.
      Basically this means that whatever the leaders of the code does the followers will do the same.
      For example, if the leftLeader is running, the leftFollower would also run.
    */

    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    /*
      rightLeader is set inverted because the right side of the robot needs to be set inverted because
      the robot would not be able to move in the same direction if the right motors were not set to 
      be inverted. For more explanation, please direct message me because it's easier to show in demonstration.
    */

    leftLeader.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightLeader.setInverted(true);
    rightFollower.setInverted(InvertType.FollowMaster);

    /**
	 * Sets the mode of operation when the robot is not doing anything.
	 *
	 * @param neutralMode
	 *            The desired mode of operation when the Controller output
	 *            throttle is neutral (ie brake/coast)
   * 
   * When commanded to coast, motor leads are set to high-impedance, allowing mechanism to coast. 
   * When commanded to brake, motor leads are commonized electrically to reduce motion. 
   * */

    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    /**
   * Construct a DifferentialDrive.
   *
   * <p>To pass multiple motors per side, use a {@link
   * edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup}. If a motor needs to be inverted, do
   * so before passing it in.
   *
   * @param leftMotor Left motor.
   * @param rightMotor Right motor.
   */

    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);   
    
    /* 
      Here you reset the gyro so that the angle the robot starts at is 0 thus making the calculations for rotating the robot 
      in any direction is easier. 
    */
    gyro.reset();
    gyro.zeroYaw();

    rightEncoder.setReverseDirection(true);
    leftEncoder.setDistancePerPulse(RobotMap.DriveTrainMap.inchesPerPulse);
    rightEncoder.setDistancePerPulse(RobotMap.DriveTrainMap.inchesPerPulse);

  }

   /**
   * Tank drive is the main driving method our team uses for driving the robot. Tank drive is part of the WPI Differential Drive
   * and more information on tank drive can be found there. For tank drive, we are able to use it as we have 2 joysticks, each 
   * controlling the power for the left motors and right motors.
   * 
   * @param left The power value you are applying to the left motor/speedcontroller. Values range from [-1.0,1.0]. The direction
   * can depend on how you set the motors up however, postitive is usally forward.
   * 
   * @param right The power value you are applying to the right motor/speedcontroller. Values range from [-1.0,1.0]. The direction
   * can depend on how you set the motors up however, postitive is usally forward.
   * 
   */

  public void tankDrive(double leftPower, double rightPower){
    differentialDrive.tankDrive(leftPower, rightPower);
    // leftLeader.set(ControlMode.PercentOutput, leftPower);
    // rightLeader.set(ControlMode.PercentOutput, rightPower);
  }

  //Our leftLeader variable is private, thus we need the getLefLeader() method to have access to the gyro outside the DriveTrain class.

  public WPI_TalonSRX getleftLeader(){
    return leftLeader;
  }

  public double getLeftLeaderEncoder(){
    return leftLeader.getSelectedSensorPosition();
  }

  //Our gyro variable is private, thus we need the getGyro() method to have access to the gyro outside the DriveTrain class.

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
