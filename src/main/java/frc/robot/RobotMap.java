package frc.robot;

public class RobotMap {
    public static class ShooterMap {
        public static final double reverseShooter = -.1;
        public static final int shootLeaderCanID = 8; //8
        public static final int shootFollowerCanID = 6; //6
        public static final int shootSolenoid = 0;
        public static final double Kp = 0.000199; //,00021
        public static final double Ki = 0;
        public static final double Kd = 0;
        public static final double Kf = 0.000080;
        public static final double lineShootRPM = 4400; 
        public static final double trenchShootRPM = 5300; 
        public static final double midTrenchShootRPM = 7150;
        public static final double maxShootRPM = 7300;
        public static final double gearRatio = .75;
    }

    public static class DriveTrainMap {
        public static final int rightLeaderCanID = 1;
        public static final int rightFollowerCanID = 2;
        public static final int leftLeaderCanID = 4;
        public static final int leftFollowerCanID = 3;

        public static final double kP = 0.052;
        public static final double kI = 0.052;
        public static final double kD = 0.052;
        public static final double izOne = 5; // la vie en rose

    }
}
