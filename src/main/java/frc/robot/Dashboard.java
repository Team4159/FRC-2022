package frc.robot;

<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.PigeonIMU;
>>>>>>> Stashed changes

public class Dashboard {


    private Arm arm;
    private Shooter shooter;
    private Feeder feeder;
    private Intake intake;
    private Climber climber;
    private Drivetrain drivetrain;

    public Dashboard (Arm, Shooter, Feeder, Intake, Climber, Drivetrain) {
        this.arm = arm;
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.climber = climber;
        this.drivetrain = drivetrain;
    }
    //Pigeon Gyro, check if it works
    public void pigeonData() {
        electrical
            .add("Gyro data", /*hope this works->*/drivetrain.getHeading())
            .withWidget(BuiltInWidgets.kGyro)
            .withSize(2, 2)
            .withPosition(4, 0);
    }
<<<<<<< Updated upstream

    public void intakeInfo() {
        SmartDashboard.putBoolean("String", true); //Example
=======
    //Encoders for arm and climber, check if they work
    public void armEncoderData(){
        electrical
            .add("Arm Encoder", arm.getEncoderRaw())
            .withWidget(BuiltInWidgets.kEncoder)
            .withSize(2, 1)
            .withPosition(4, 2)
            .getEntry();
>>>>>>> Stashed changes
    }

    public void climberEncoderData(){
        electrical
            .add("Climber Encoder", climber.getEncoderRaw())
            .withWidget(BuiltInWidgets.kEncoder)
            .withSize(2, 1)
            .withPosition(4, 3)
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
    public void PDP_Voltage(){
        electrical
            //.add("PDP", )
            .withWidget(BuiltInWidgets.kPowerDistribution)
            .getVoltage();
    }

    public void PDP_TotalCurrent(){
        electrical
            .withWidget(BuiltInWidgets.kPowerDistribution)
            .getTotalCurrent();

        double leftJoystickCurr = leftJoystickPort.getCurrent();
    }
}
