package frc.robot;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.commands.Climb;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RunFeeder;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunNeck;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.commands.CommandGroups.NeckAndShoot;
import frc.robot.subsystems.Arm.ArmState;
// import frc.robot.subsystems.Climber.ClimberState;


public class Dashboard {
    private RobotContainer robotContainer;
    private ShuffleboardTab electrical = Shuffleboard.getTab("Electrical");

    //private NetworkTableEntry leftJoystick = electrical.add("Left Joystick", 0).getEntry();
    //private NetworkTableEntry rightJoystick = electrical.add("Right Joystick", 0).getEntry();
    
    // NetworkTableEntry xEntry;
    // NetworkTableInstance shooterVelocity = NetworkTableInstance.getDefault();
    // NetworkTable table = shooterVelocity.getTable("datatable");

    // NetworkTable table = shooterVelocity.getTable("datatable");
    // xEntry = table.getEntry("X");
    // xEntry.setDouble(m_robotContainer.getShooter().getVelocity());

    

    //Call methods here to run them so they do their thing in shuffleboard
    public Dashboard (RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void update() {
        // encoderValues();
        getJoystickInputs();
        // getMotorOutputs();
        // commands();
        // pigeonData();
        // powerDistData();
        //xEntry.setDouble(robotContainer.getShooter().getVelocity());
    }

    public void getJoystickInputs(){
        //     .add("Left Joystick", robotContainer.getLJoystick().getY());
            // .withWidget(BuiltInWidgets.kNumberBar)
            // .withSize(1,1)
            // .withPosition(0,0)
            // .getEntry();
        // Shuffleboard.getTab("Electrical")
        //     .add("Right Joystick", robotContainer.getRJoystick().getY())
        //     .withWidget(BuiltInWidgets.kNumberBar)
        //     .withSize(1,1)
        //     .withPosition(1,0);

        double leftJoystickInput = -robotContainer.getLJoystick().getY();
        //leftJoystick.setDouble(leftJoystickInput);

        double rightJoystickInput = -robotContainer.getRJoystick().getY();
        //rightJoystick.setDouble(rightJoystickInput);


    }
    // public void getMotorOutputs(){
    //     Shuffleboard.getTab("Electrical")
    //         .add("Intake", robotContainer.getIntake().getIntakeSpark().get())
    //         .withWidget(BuiltInWidgets.kNumberBar)
    //         .withSize(1, 1)
    //         .withPosition(0, 4);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Feeder", robotContainer.getFeeder().getFeederSpark().get())
    //         .withWidget(BuiltInWidgets.kNumberBar)
    //         .withSize(1, 1)
    //         .withPosition(1, 4);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Neck", robotContainer.getNeck().getNeckSpark().get())
    //         .withWidget(BuiltInWidgets.kNumberBar)
    //         .withSize(1, 1)
    //         .withPosition(2, 4);
    //     //Arm outputs
    //     Shuffleboard.getTab("Electrical")
    //         .add("Arm Outputs", robotContainer.getArm().getArmSpark().get())
    //         .withWidget(BuiltInWidgets.kNumberBar)
    //         .withSize(1, 1)
    //         .withPosition(8, 1);
    //     //Climber outputs
    //     Shuffleboard.getTab("Electrical")
    //         .add("Climber Outputs", robotContainer.getClimber().getClimberGroup().get())
    //         .withWidget(BuiltInWidgets.kNumberBar)
    //         .withSize(1, 1)
    //         .withPosition(9, 1);
    // }

    // public void commands() {
    //     Shuffleboard.getTab("Electrical")
    //         .add("Run Intake", new RunIntake(robotContainer.getIntake(), frc.robot.Constants.Direction.FORWARDS))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(0, 3);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Run Feeder", new RunFeeder(robotContainer.getFeeder(), frc.robot.Constants.Direction.FORWARDS))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(1, 3);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Run Neck", new RunNeck(robotContainer.getNeck(), frc.robot.Constants.Direction.FORWARDS))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(2, 3);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Arm, Intake, and Feeder", new ArmIntakeAndFeeder(robotContainer.getArm(), robotContainer.getIntake(), robotContainer.getFeeder()))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(3, 3);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Neck and Shoot", new NeckAndShoot(robotContainer.getFeeder(), robotContainer.getNeck(), robotContainer.getShooter()))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(4, 3);
    //     //Arm commands
    //     Shuffleboard.getTab("Electrical")
    //         .add("Raise Arm", new MoveArm(robotContainer.getArm(), ArmState.HIGH))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(8, 0);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Lower Arm", new MoveArm(robotContainer.getArm(), ArmState.LOW))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(8, 2);
    //     //Climber commands
    //     Shuffleboard.getTab("Electrical")
    //         .add("Raise Climber", new Climb(robotContainer.getClimber(), ClimberState.RAISE))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(9, 0);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Lower Climber", new Climb(robotContainer.getClimber(), ClimberState.LOWER))
    //         .withWidget(BuiltInWidgets.kCommand)
    //         .withSize(1,1)
    //         .withPosition(9, 2);
    // }
    public void pigeonData() {
        // Shuffleboard.getTab("Electrical")
        //     .add("Gyro data", robotContainer.getDriveTrain().getHeading())
        //     .withWidget(BuiltInWidgets.kGyro)
        //     .withSize(2, 2)
        //     .withPosition(0, 1);
        //double gyroOutput = robotContainer.getDriveTrain().getAngle();
        //gyro.setDouble(gyroOutput);
    }

    // public void encoderValues() {
    //     Shuffleboard.getTab("Electrical")
    //         .add("Arm Encoder", robotContainer.getArm().getEncoder())
    //         .withWidget(BuiltInWidgets.kEncoder)
    //         .withSize(3, 1)
    //         .withPosition(4, 0);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Climber Encoder", robotContainer.getClimber().getEncoder())
    //         .withWidget(BuiltInWidgets.kEncoder)
    //         .withSize(2, 1)
    //         .withPosition(8, 3);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Left Front Talon Encoder", robotContainer.getDriveTrain().getLeftTalon())
    //         .withWidget(BuiltInWidgets.kEncoder)
    //         .withSize(2, 1)
    //         .withPosition(4, 4);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Right Front Talon Encoder", robotContainer.getDriveTrain().getRightTalon())
    //         .withWidget(BuiltInWidgets.kEncoder)
    //         .withSize(2, 1)
    //         .withPosition(7, 4);
    //     Shuffleboard.getTab("Electrical")
    //         .add("Shooter", robotContainer.getShooter().getVelocity())
    //         .withWidget(BuiltInWidgets.kGraph)
    //         .withSize(3, 3)
    //         .withPosition(2, 0);
    // }

    // //PDP code
    // public void powerDistData(){
    //     Shuffleboard.getTab("Electrical")
    //     .add("Power Distribution Hub", robotContainer.getPDP())
    //     .withWidget(BuiltInWidgets.kPowerDistribution)
    //     .withSize(3, 2)
    //     .withPosition(6,1);
    // }

}