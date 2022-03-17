package frc.robot;

public class RobotMap {
    
    public static class DriveTrainMap{
        public static final double angleKP = 0.1945392;
        public static final double angleKI = 0;
        public static final double angleKD = 0.00323242;
        public static int leftLeaderCANID = 1;
        public static int leftFollowerCANID = 2;
        public static int rightLeaderCANID = 3;
        public static int rightFollowerCANID = 4;

        public static double radiusOfWheel = 3;
        //Radius of the wheel of the robot. (In terms of inches)
        
        public static double inchesPerPulse = 0.12058884343625044337347446800602; // MAJIK NUMBER WORKS DO NOT CHANGE (maybe take off a few decimal places)
        // 0.78858884343625044337347446800602; // old value
        // public static double inchesPerPulse = (1.0 / 20) * (1.0 / 10.75) * (6.0 * 3.14159);
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
        public static final double elevatorMaxHeight = 379;
        public static final double elevatorMinHeight = 20;

        public static final double climberMotorUp = 1;
        public static final double climberMotorDown = -1;
        public static final double elevatorMotorUp = 0.25;
        public static final double elevatorMotorDown = -0.25;
        
        public static final double elevatorHalt = 0.0;
        // public static final int upperLimit = 193; none of these are used
        // public static final int lowerLimit = 2;
        public static final int modNumSolenoid = 0;
    }
    
    public static class ShooterMap {
        public static final double gearRatio = 0.75;
        public static final int ShooterLeaderCANID = 6;
        public static final int ShooterFollowerCANID = 7;
        public static final double kP = 0.00081675; //0.00061675
        public static final double kD = 0.52171675; //0.051675
        public static final double shooterPowerShort = 0.5;
        public static final double shooterPowerLong = 0.75;
        public static double ballCurrent = 15;

        public static final double kV = 0.0021363;
        public static final double KA = 0.0106166;
    }
    
    public static class IntakeMap {
        public static int intakeMotorCANID = 5;
        public static int intakeDeployMotorCANID = 8;
        public static double intakeMotorPower = -0.8;
        public static int deployEncoderChannel1 = 4;
        public static int deployEncoderChannel2 = 5;
        public static double deployMotorPower = 1;

        //0.735-0.745 Start
        public static double voltageValueForUndeployedLower = 0.735;
        public static double voltageValueForUndeployedUpper = 0.75;

        //0.37-0.38
        public static double voltageValueForDeployedLower = 0.37;
        public static double voltageValueForDeployedUpper = 0.375;
    }

    public static class LoaderMap{
        public static int loaderMotorCANID = 9;
    }

}
