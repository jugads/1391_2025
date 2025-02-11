package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public class DrivetrainConstants {
        public static final double kMaxSpeed = 5.41;
        public static final double kMaxAngularRate = kMaxSpeed * 39.37 / 20.75 * Math.PI;
    }
    public class OperatorConstants {
        public static final int kL4 = 1;
        public static final int kL3 = 2;
        public static final int kL2 = 3;
        public static final int kL1 = 4;
        public static final int kAutoAlignLeft = 9;
        public static final int kAutoAlignRight = 8;
        public static final int k0degrees = 10;
        public static final int k60degrees = 12;
        public static final int k120degrees = 6;
        public static final int k180degrees = 5;
        public static final int k240degrees = 7;
        public static final int k300degrees = 11;
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
        public static final double kI = 0.0028;
        public static final double kD = 0.17;
        public static final double kPDynamic = 0.2;
        public static final double kIDynamic = 0.03;
        public static final double kDDynamic = 0;
       
    }
    public class ArmConstants{
        public static final int kMotorID = 3;
        public static final int kEncoderPort = 8;
        public static final double kP = 0.17;
        public static final double kI = 0.02;
        public static final double kD = 0;
        public static final double kTransferAngle = 0;
        public static final double kPDynamic = 0.006;
        public static final double kIDynamic = 0.0;
        public static final double kDDynamic = 0.00035;
    }
    public class ReefPoses {
        public static final Pose2d kRED0_1 = new Pose2d(11.1, 4.25, Rotation2d.fromDegrees(0));
        public static final Pose2d kRED2_3 = new Pose2d(12, 5.6, Rotation2d.fromDegrees(-60));
        public static final Pose2d kRED4_5 = new Pose2d(14, 5.7, Rotation2d.fromDegrees(-120));
        public static final Pose2d kRED6_7 = new Pose2d(15, 4, Rotation2d.fromDegrees(180));
        public static final Pose2d kRED8_9 = new Pose2d(14, 2.5, Rotation2d.fromDegrees(120));
        public static final Pose2d kRED10_11 = new Pose2d(12.14, 2.43, Rotation2d.fromDegrees(60));
        public static final Pose2d kREDSOURCERIGHT = new Pose2d(16.26, 7.1, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCELEFT = new Pose2d(16.42, 1, Rotation2d.fromDegrees(128));
        public static final PathConstraints K_CONSTRAINTS = new PathConstraints(3.0, 3.0, 2*Math.PI, 4*Math.PI);
    }
}
