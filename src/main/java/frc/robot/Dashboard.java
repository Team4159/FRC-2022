package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.Arm;
//import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Feeder;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shooter;

//import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.PigeonIMU;

public class Dashboard {
    private RobotContainer robotContainer;

    //Call methods here to run them so they do their thing in shuffleboard
    public Dashboard (RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void update() {
        //test();
        //pigeonData();
        armEncoderData();
        getJoystickInputs();
        //climberEncoderData();

        //lJoystickPort();
        //rJoystickPort();
        //secJoystickPort();

        System.out.println("update");
    }

    public void getJoystickInputs(){
        Shuffleboard.getTab("Electrical")
            .add("Left Joystick", robotContainer.getLJoystick().getY())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withSize(1,1)
            .withPosition(0,0);
        Shuffleboard.getTab("Electrical")
            .add("Right Joystick", robotContainer.getRJoystick().getY())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withSize(1,1)
            .withPosition(1,0);
    }
    public void getMotorOutputs(){
        Shuffleboard.getTab("Electrical")
            .add("Intake", robotContainer.getIntake().getIntakeSpark().get());
    }
    public void pigeonData() {
        Shuffleboard.getTab("Electrical")
            .add("Gyro data", robotContainer.getDriveTrain().getRotation())
            .withWidget(BuiltInWidgets.kGyro)
            .withSize(2, 2)
            .withPosition(0, 1);
    }
    //Encoders for arm and climber, check if they work
    public void armEncoderData(){
        SmartDashboard.putNumber("Arm Encoder", robotContainer.getArm().getEncoderRaw());
        Shuffleboard.getTab("Electrical")
            .add("Arm Encoder", robotContainer.getArm().getEncoderRaw())
            .withWidget(BuiltInWidgets.kEncoder)
            .withSize(2, 1)
            .withPosition(4, 2)
            .getEntry();
    }

    public void climberEncoderData(){
        SmartDashboard.putNumber("Climber Encoder", robotContainer.getClimber().getEncoderRaw());
        Shuffleboard.getTab("Electrical")
            .add("Climber Encoder", robotContainer.getClimber().getEncoderRaw())
            .withWidget(BuiltInWidgets.kEncoder)
            .withSize(2, 1)
            .withPosition(4, 3)
            .getEntry();
    }
    //PDP code
    public void powerDistData(){
        SmartDashboard.putNumber("PDP Data", robotContainer.getPDP().getTotalCurrent());
        Shuffleboard.getTab("Electrical")
        .add("PDP Data", robotContainer.getPDP().getTotalCurrent())
        .withWidget(BuiltInWidgets.kPowerDistribution)
        .withSize(3, 2)
        .withPosition(0,0)//temporary position
        .getEntry();
    }

    //Encoders for all motors
    public void motorEncoder1 () {
        //stuff
    }

    /*
    public void intakeEncoderData(){
        electrical
            .withWidget(BuiltInWidgets.kEncoder)
            .
    }
    */

    //PDP Current Stuff
        //Left Joystick
/*
    public void lJoystickPort () {
        Shuffleboard.getTab("Electrical")
        //getVoltage returns voltage for whole robot, not a specific port. 
        //what exactly is wanted to be put here?
            .add("Left Joystick Current", robotContainer.PDH.getCurrent(0))
            .withWidget(BuiltInWidgets.kPowerDistribution);
    }
        //Right Joystick
    public void rJoystickPort () {
        Shuffleboard.getTab("Electrical")
            .add("Right Joystick", robotContainer.PDH.getCurrent(1))
            .withWidget(BuiltInWidgets.kPowerDistribution);
    }
        //Secondary Joystick
    public void secJoystickPort () {
        Shuffleboard.getTab("Electrical")
            .add("Right Joystick", robotContainer.PDH.getCurrent(2))
            .withWidget(BuiltInWidgets.kPowerDistribution);
    }
        //Pigeon gyro 
    public void pigeonPDP () {
        Shuffleboard.getTab("Electrical")
            .add("Pigeon Current", robotContainer.PDH.getCurrent(0))
            .withWidget(BuiltInWidgets.kPowerDistribution);
    }
        //Right front talon
    public void rTalonFrontPDP () {
        Shuffleboard.getTab("Electrical")
            .add("Right Front Talon Current", robotContainer.PDH.getCurrent(1))
            .withWidget(BuiltInWidgets.kPowerDistribution);
    }*/

    /*public void PDP_TotalCurrent(){
        electrical
            .withWidget(BuiltInWidgets.kPowerDistribution)
            .getTotalCurrent();

        double leftJoystickCurr = leftJoystickPort.getCurrent();
    }*/
    /*public String green (){
        Shuffleboard.getTab("Electrical")
        .add("Text View", "oo")
        .withWidget("Text View")
        .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
        .getEntry();
    }
    public void test (){
        SmartDashboard.putString("Text View", green());
    }*/
}