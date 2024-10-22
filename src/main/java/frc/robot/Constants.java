package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    
    public static class TurretConstants {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;
        
    
        public static Rotation2d kMaxVelocity = Rotation2d.fromDegrees(135);
        public static Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(135);
    
    }

}
