package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotMap {

    public static class ElevatorMap{

        //most are not actual values, will change later when I get a confirmed number for any (most are based off of previous year code)
        public static final int elevatorCANID = 8;
        public static final int topLimitChannelID = 1;
        public static final int bottomLimitChannelID = 2;
        public static final int elevatorSolenoid = 2;
        public static final double elevatorMotorUp = 0.5;
        public static final double elevatorMotorDown = -0.5;
        public static final double elevatorHalt = 0.0;
        public static final int upperLimit = 193;
        public static final int lowerLimit = 2;
        public static final int modNumSolenoid = 0;
    }
    
    public static class DriveTrainMap{
        public static int leftLeaderCANID = 1;
        public static int leftFollowerCANID = 4;
        public static int rightLeaderCANID = 2;
        public static int rightFollowerCANID = 3;

        /*CAN-ID VALUES FOR OUR DRIVETRAIN MOTOR CONTROLLORS. CAN-ID values are values we assign to motors for our
        RoboRio to be able to use our motors and know which motors motor controllor needs to be used. 
        */

        public static double radiusOfWheel = 3;
        //Radius of the wheel of the robot. (In terms of inches)
        
        public static double ticksToInches = 0.35190777138342676035541298134768;
        //0.04928680271 for quadature.
      
        /*
        When getting values from motors, like encoder ticks, we need to convert those ticks into actual measurement. 
        Thus we convert those tick values into actual values using this constant. The (43/7) is the gear ratio and 
        can be represented in a ratio like 43:7. You need to include the gear ratio into your code and typically 
        we'll be given the gear ratios by the manufacturing/design departments. 
        */

    }
    
    public static class ShooterMap {
        public static final double gearRatio = 0.75;
        public static final int ShooterLeaderCANID = 16;
        public static final int ShooterFollowerCANID = 7;
        public static final double kP = 0.00061675; //0.00061675
        public static final double kD = 0.071675; //0.051675

    }
    
    public static class IntakeMap
    {
        public static int intakeMotorCANID = 4;
        public static int intakeDeployMotorCANID = 5;
        public static double intakeMotorPower = 0.6;
        public static int deployEncoderChannel1 = 4;
        public static int deployEncoderChannel2 = 5;
        public static double deployMotorPower = 0.4;
    }

}
