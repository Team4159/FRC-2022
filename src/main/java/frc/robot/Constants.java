package frc.robot;

import edu.wpi.first.math.util.Units;
import java.lang.Math;

public class Constants {

    public class CanIds {

        public final static int pigeonId = 0;
        public final static int rightFrontTalon = 1;
        public final static int rightRearTalon = 2;
        public final static int leftFrontTalon = 3;
        public final static int leftRearTalon = 4;

        public final static int intakeSpark1 = 5;
        public final static int intakeSpark2 = 6;
        public final static int feederSpark1 = 7;
        public final static int feederSpark2 = 8;
        public final static int feederSpark3 = 9;
        public final static int climberSpark1 = 10;
        public final static int climberSpark2 = 11;
        public final static int shooterSpark1 = 12;
        public final static int shooterSpark2 = 13;
    }

    public static class JoystickConstants {
        public final static int leftJoystickPort = 0;
        public final static int rightJoystickPort = 1;
        public final static int secondaryJoystickPort = 2;

        public static class SecondaryJoystick { // Temporary Buttons for Controls meant for Week 1 Teams to use
            public final static int lowerArm = 0; // Implement PID System to preset these positions
            public final static int raiseArm = 1;
            public final static int runIntakeForward = 2;
            public final static int runIntakeBackwards = 3;
            public final static int runFeeder = 4;
            public final static int runShooter = 5;
        }

    }

    public static class DriveTrainConstants {

        public final static double metersPerRev = Units.metersToInches(3) * 2 * Math.PI;

        public final static double trackWidth = 0.5; // TODO: Need To Determine In Meters

    }

}
