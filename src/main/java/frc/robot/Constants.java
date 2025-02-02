package frc.robot;

public class Constants {
    public class DrivetrainConstants {
        public static final double kMaxSpeed = 5.41;
        public static final double kMaxAngularRate = kMaxSpeed * 39.37 / 20.75 * Math.PI;
    }
    public class KnuckleConstants {
        public static final int kMotorID = 5;
        // public static final double kCurrentThreshold = 75;
        public static final double kHighSpeed = -0.4;
        public static final double kLowSpeed = -0.05;
    }
    public class ChuteConstants {
        public static final int kMotorID = 8;
        public static final double kCurrentThreshold = 30;
        public static final int kBeamBreakPort = 0;
    }
    public class AlgaeScorerConstants{
        public static final int kMotorID = 4;
        public static final double kCurrentThreshold = 20;
        
    }
    public class ElevatorConstants{
        public static final int kLeftMotorID = 1;
        public static final int kRightMotorID = 2;
        // public static final int kDownLimitPort = 0;
        // public static final int kUpLimitPort = 0;
        public static final double kP = 3.8;
        public static final double kI = 0.0025;
        public static final double kD = 0.17;
        public static final double kPDynamic = 0.2;
        public static final double kIDynamic = 0.02;
        public static final double kDDynamic = 0;
       
    }
    public class ArmConstants{
        public static final int kMotorID = 3;
        public static final int kEncoderPort = 9;
        public static final double kP = 0.17;
        public static final double kI = 0.01;
        public static final double kD = 0;
        public static final double kTransferAngle = 0;
        public static final double kPDynamic = 0.006;
        public static final double kIDynamic = 0.0;
        public static final double kDDynamic = 0.0003;
    }
}
