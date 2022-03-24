package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class RobotMap {
    public static class DriveTrainMap {
        public static final int rightLeaderCanID = 4;
        public static final int rightFollowerCanID = 3;
        public static final int leftLeaderCanID = 1;
        public static final int leftFollowerCanID = 2;

        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.8994; //0.6384
        public static final double kvVoltSecondsPerMeter = 1.9714; //0.023005
        public static final double kaVoltSecondsSquaredPerMeter = 1.7399; //0.0014474

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 6.1059;  //0.061451
        public static final double kTrackwidthMeters = 0.6604;

        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;

        public static final double ticksToMeters = 0.000023482232685981534717425546180214;
        //2.3482232685981534717425546180214e-5 for ticks to Meters
        //0.4787787204070844895417068516118 is the Circumference of the Robot in meters.

        public static double inchesPerPulse = 0.12058884343625044337347446800602; 
        
        public static final DifferentialDriveKinematics dKINEMATICS = new DifferentialDriveKinematics(kTrackwidthMeters);

    }

    public static class ElevatorMap {

    }

    public static class IntakeMap {

    }

    public static class ShooterMap {
        
    }
}
