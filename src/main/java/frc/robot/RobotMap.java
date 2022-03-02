package frc.robot;

public class RobotMap {
    
    public static class DriveTrainMap{
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static int leftLeaderCANID = 1;
        public static int leftFollowerCANID = 2;
        public static int rightLeaderCANID = 3;
        public static int rightFollowerCANID = 4;

        public static double radiusOfWheel = 3;
        //Radius of the wheel of the robot. (In terms of inches)
        
        public static double inchesPerPulse = 0.78858884343625044337347446800602;
        //0.04928680271 for quadature.
    }

    public static class ElevatorMap{

        //most are not actual values, will change later when I get a confirmed number for any (most are based off of previous year code)
        public static final int elevatorCANID = 10;
        public static final int elevatorTurningLeader = 11;
        public static final int elevatorTurningFollower = 12;
        public static final int topLimitChannelID = 1;
        public static final int bottomLimitChannelID = 2;
        public static final int elevatorSolenoid = 2;
        public static final double elevatorMaxHeight = 232;

        public static final double elevatorMotorUp = 0.5;
        public static final double elevatorMotorDown = -0.5;
        
        public static final double elevatorHalt = 0.0;
        public static final int upperLimit = 193;
        public static final int lowerLimit = 2;
        public static final int modNumSolenoid = 0;
    }
    
    public static class ShooterMap {
        public static final double gearRatio = 0.75;
        public static final int ShooterLeaderCANID = 6;
        public static final int ShooterFollowerCANID = 7;
        public static final double kP = 0.00061675; //0.00061675
        
        public static final double kD = 0.09171675; //0.051675
        public static final double shooterPowerShort = 0.5;
        public static final double shooterPowerLong = 0.75;
        public static double ballCurrent = 5;
    }
    
    public static class IntakeMap
    {
        public static int intakeMotorCANID = 5;
        public static int intakeDeployMotorCANID = 8;
        public static double intakeMotorPower = -0.6;
        public static int deployEncoderChannel1 = 4;
        public static int deployEncoderChannel2 = 5;
        public static double deployMotorPower = 0.4;

        //0.735-0.745 Start
        public static double voltageValueForUndeployedLower = 0.735;
        public static double voltageValueForUndeployedUpper = 0.75;

        //0.37-0.38
        public static double voltageValueForDeployedLower = 0.37;
        public static double voltageValueForDeployedUpper = 0.387;


    }

    public static class LoaderMap{
        public static int loaderMotorCANID = 9;
    }

}
