package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;


/*
    Runs a ~ 30 second test before we switch the battery after every match to ensure all subsystems + commands are running correctly
    Test once you enable the test mode in Driverstation
*/

public class Test extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Intake intake;
    private Arm arm;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;
    private Timer timer = new Timer();
    private double outputValue;
    private double timesBroke; 
    private String fail = "";
    
    public Test(Drivetrain drivetrain, Intake intake, Arm arm, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;
        timer.start();
        timesBroke = 0;
    }

    public void Test() {
        if(timer.get() < 1) {
            intake.runIntake(Direction.FORWARDS);
            System.out.println(tolerance(intake.getIntakeSpark().get(), Constants.IntakeAndArmConstants.intakeSpeed, 0.01));
            if(!tolerance(intake.getIntakeSpark().get(), Constants.IntakeAndArmConstants.intakeSpeed, 0.01)) {
                timesBroke ++;
            }
        }
        else if(timer.get() == 1) {
            if(timesBroke > 0) {
                System.out.print("------------------------------");
                System.out.println("Intake Forwards: " + "\u2715");
                System.out.print("------------------------------");
                fail += "Intake Forwards, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Intake Forwards: " + "\u2713");
                System.out.print("------------------------------");
            }
        }
        else if(timer.get() < 2) {
            intake.runIntake(Direction.BACKWARDS);
            System.out.println(tolerance(intake.getIntakeSpark().get(), -Constants.IntakeAndArmConstants.intakeSpeed, 0.01));
            if(!tolerance(intake.getIntakeSpark().get(), -Constants.IntakeAndArmConstants.intakeSpeed, 0.01)) {
                timesBroke ++;
            }
        }
        else if(timer.get() == 2) {
            if(timesBroke > 0) {
                System.out.print("------------------------------");
                System.out.println("Intake Backwards: " + "\u2715");
                System.out.print("------------------------------");
                fail += "Intake Backwards, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Intake Backwards: " + "\u2713");
                System.out.print("------------------------------");
            }
        }
        else if(timer.get() < 3) {
            arm.runArm(ArmState.HIGH);
            if(arm.atSetpoint(Constants.IntakeAndArmConstants.pidHighSetPoint, Constants.IntakeAndArmConstants.tolerance)) {
                
            }
        }
        else if(timer.get() == 3) {
            if(timesBroke > 0) {
                System.out.println("Intake Backwards: " + "\u2715");
                fail += "Intake Backwards, ";
            }
            else {
                System.out.print("Intake Backwards: " + "\u2713");
            }
        }
    }

    public boolean tolerance(double value, double expectedValue, double tolerance) {
        return value <= expectedValue + tolerance && value >= expectedValue - tolerance;
    }
}
