package frc.robot;

public class RobotMap {
    public static class ShooterMap {
        public static final double gearRatio = 0.75;
        public static final int ShooterLeaderCANID = 6;
        public static final int ShooterFollowerCANID = 7;
        public static final double kP = 0.00061675; //0.00061675
        
        public static final double kD = 0.09171675; //0.051675
        public static final double shooterPowerShort = 0.5;
        public static final double shooterPowerLong = 1;
        public static double ballCurrent = 5;
    }

    public static class DriveTrainMap {
        public static int leftLeaderCANID = 1;
        public static int leftFollowerCANID = 2;
        public static int rightLeaderCANID = 3;
        public static int rightFollowerCANID = 4;
        
        public static final double angleKp = 0.052;
        public static final double angleKi = 0.0;
        public static final double angleKd = 0.0;

        public static final double distKp = 0.0;
        public static final double distKi = 0.0;
        public static final double distKd = 0.0;

        public static final double izOne = 5; 
    }
    
}
