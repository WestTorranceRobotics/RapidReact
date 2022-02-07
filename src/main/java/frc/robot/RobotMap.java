package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class RobotMap {
    public static class DriveTrainMap {
        public static final int rightLeaderCanID = 1;
        public static final int rightFollowerCanID = 2;
        public static final int leftLeaderCanID = 4;
        public static final int leftFollowerCanID = 3;

        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.66904; //0.6384
        public static final double kvVoltSecondsPerMeter = 0.022005; //0.023005
        public static final double kaVoltSecondsSquaredPerMeter = 0.002674; //0.0014474

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.029217;  //0.061451
        public static final double kTrackwidthMeters = 0.6604;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double ticksToMeters = 0.04470389546284635756691940724667;

        public static final DifferentialDriveKinematics dKINEMATICS = new DifferentialDriveKinematics(kTrackwidthMeters);

    }

    public static class ElevatorMap {

    }

    public static class IntakeMap {

    }

    public static class ShooterMap {
        
    }
}
