package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import java.lang.Math;

import org.w3c.dom.views.DocumentView;

public class Constants {

    public class CanIds {
      
        public final static int pigeonId = 0;
        public final static int rightFrontTalon = 1;
        public final static int rightRearTalon = 2;
        public final static int leftFrontTalon = 3;
        public final static int leftRearTalon = 4;
        public final static int shooterTalonRight = 5;
        public final static int shooterTalonLeft = 6;
        public final static int intakeSpark = 13;
        public final static int armSpark1 = 10;
        public final static int armSpark2 = 12;
        public final static int feederSpark = 9;
        public final static int neckSpark = 7;
        public final static int climberSpark1 = 8;
        public final static int climberSpark2 = 11;
        
    }

    public class JoystickConstants {
        public final static int leftJoystickPort = 0;
        public final static int rightJoystickPort = 1;
        public final static int secondaryJoystickPort = 2;

        public class SecondaryJoystick { 
            public final static int lowerArm = 13; 
            public final static int raiseArm = 14;
            public final static int runIntakeForwards = 12;
            public final static int runIntakeBackwards = 15;

            public final static int runFeederForwards = 7;
            public final static int runFeederBackwards = 8;
            
            public final static int raiseClimber = 4;
            public final static int lowerClimber = 3;
            public final static int runNeck = 6;
            public final static int runNeckBackwards = 9;

            public final static int runNeckAndShoot = 1;
            public final static int moveArmIntakeAndFeed = 2;

            //public final static int runShooter = 10;
        }
    }

    public static class DriveTrainConstants {
        private final static double kP = 0.5;
        private final static double kI = 0;
        private final static double kD = 0;

        public final static double encoderEdgesPerRev = 2048;
        public final static double gearRatio = 8.667;
        public final static double wheelCircumference = 2 * Math.PI * Units.inchesToMeters(3);

        public final static double trackWidth = Units.inchesToMeters(25);
        public static final DifferentialDriveKinematics kDriveKinematics =new DifferentialDriveKinematics(trackWidth);

        
        //TODO: Need to get actual values later from the characterization tool.
        public static final double ksVolts = 0.771205;
        public static final double kvVoltSecondsPerMeter = 1.96575;
        public static final double kaVoltSecondsSquaredPerMeter = 0.681715;

        public static final double kPDriveVel = 3.06255;

        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

        //Officially Tested 
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    
    public static class ClimberConstants {
        public final static int encoderChannelA = 2;
        public final static int encoderChannelB = 3;
        public final static Boolean encoderReverse = false;
        public final static EncodingType encodingType = EncodingType.k1X;

        public final static double kP = 0;
        public final static double kI = 0;
        public final static double kD = 0;

        public final static int pidLowSetPoint = 0;
        public final static int pidHighSetPoint = 0;
        public final double trackWidth = Units.inchesToMeters(25); // TODO: Need To Determine In Meters


        //Need to get actual values later from the characterization tool.
        public static final  double kS = 1;
        public static final double kV = 1;
        public static final double kA = 1;

    }

    public static enum Direction {
        FORWARDS,
        BACKWARDS
    }

    public static class IntakeAndArmConstants {
        public final static double intakeSpeed = 0.6;
        public final static double backwardsIntakeSpeed = -0.6;
        
        public final static double raiseArmSpeed = 0.1;
        public final static double lowerArmSpeed = -0.1;

        public final static int encoderChannelA = 0;
        public final static int encoderChannelB = 1;
        public final static boolean encoderReverse = false;
        public final static EncodingType encodingType = EncodingType.k1X;

        public final static double kP = 0.00022; //Don't touch these constants unless yaknow whatcha doing
        public final static double kI = 0;
        public final static double kD = 0.00000001;
        public final static double tolerance = 100;
        
        public final static double pidLowSetPoint = 2200;
        public final static double pidHighSetPoint = 0;
    }

    public static class FeederConstants {
        public final static double feederSpeed = 1;
        public final static double backwardsFeederSpeed = -1;
    }

    public static class NeckConstants {
        public final static double neckSpeed = 1;
        public final static double backwardsNeckSpeed = -1;
    }

    public static class ShooterConstants {
        public final static int kPIDLoopIdx = 0;
        public final static double kP = 1;
        public final static double kI = 0.1;
        public final static double kD = 0.3;
        public final static double kF = 0;

        public final static double targetVelocity = 21000;
    }

}