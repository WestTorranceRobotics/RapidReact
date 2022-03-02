// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  //motor for elevator may be victor spx or falcon 500
  private CANSparkMax ElevatorMotor;
  private CANSparkMax ElevatorTurningLeader;
  private CANSparkMax ElevatorTurningFollower;
  private DigitalInput TopLimit;
  private DigitalInput BottomLimit;
  private Solenoid BreakOff;

  /** Creates a new Elevator. */
  public Elevator() {
    //declaring motor and limits to their can ids
    ElevatorMotor = new CANSparkMax(RobotMap.ElevatorMap.elevatorCANID, MotorType.kBrushless);
    ElevatorTurningLeader = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    ElevatorTurningFollower = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningFollower, MotorType.kBrushless);
    // TopLimit = new DigitalInput(RobotMap.ElevatorMap.topLimitChannelID);
    // BottomLimit = new DigitalInput(RobotMap.ElevatorMap.bottomLimitChannelID);
    // BreakOff = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.ElevatorMap.elevatorSolenoid);
    ElevatorTurningFollower.follow(ElevatorTurningLeader);
    // ElevatorMotor.setIdleMode(IdleMode.Brake);

    ElevatorMotor.setInverted(true);
    ElevatorMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CANSparkMax getElevatorMotor(){
    return ElevatorMotor;
  }

  public double getElevatorMotorTicks(){
    return ElevatorMotor.getEncoder().getPosition();
  }

  /*victorspx requires controlmode value (all options can be seen with ctrl + click on ControlMode)
    typically we use percent output, position is sometimes used for drivedistance, velocity is also sometimes used alongside pid
    most libraries do not ask for a control mode*/

  //lifts up elevator 
  public void liftUp(){
    ElevatorMotor.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  //lowers down elevator
  public void liftDown(){
    ElevatorMotor.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void liftForwards(){
    ElevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void liftBackwards(){
    ElevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  //stops elevator
  public void setNoPower(){
    ElevatorMotor.set(RobotMap.ElevatorMap.elevatorHalt);
    ElevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorHalt);
    ElevatorTurningFollower.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  //notifies when top is reached
  public boolean reachedTopLimit(){
    return TopLimit.get();
  }

  //notifies when bottom is reached
  public boolean reachedBottomLimit(){
    return BottomLimit.get();
  }
}
