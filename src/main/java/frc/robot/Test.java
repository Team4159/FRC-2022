package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Direction;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;


/*
    Runs a ~ 30 second test before we switch the battery after every match to ensure all subsystems + commands are running correctly
    Test once you enable the test mode in Driverstation
*/

public class Test extends SequentialCommandGroup{
    private MoveDistance moveDistance;
    private Drivetrain drivetrain;
    private Intake intake;
    private Arm arm;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;
    private Timer timer = new Timer();
    private double outputValue;
    private String fail = "";
    
    public Test(Drivetrain drivetrain, Intake intake, Arm arm, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;
        this.moveDistance = MoveDistance(drivetrain, 3); //TODO: figure out how far that 3 is
        timer.start();
    }

    public void Test() {
        //Ordered based on the order in vscode (alphabetical inside the robot folder)
        //AutoCommands
        if(timer.get() < 1) {
            moveDistance.execute();
        }
        else if(timer.get() == 1) {
            if(!moveDistance.isFinished()) { //TODO: isFinished might run early and could give false results but without the robot I can't test it
                System.out.println("------------------------------");
                System.out.println("MoveDistance: BROKEN \u2715");
                fail += "MoveDistance, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("MoveDistance: OK \u2713");
            }
        }
        
        //below stuff is kinda broken probably maybe not?
        //I won't touch it for now but I'll edit it once I get back to subsystems
        
        if(timer.get() < 1) {
            intake.runIntake(Direction.FORWARDS);
            //spin up motor
            // System.out.println(tolerance(intake.getIntakeSpark().get(), Constants.IntakeAndArmConstants.intakeSpeed, 0.01));
            //^ will get annoying after a while, might be good for debugging though
        }
        else if(timer.get() == 1) {
            //give the motor time to spin up and then check if the final speed is within expected levels
            if(!tolerance(intake.getIntakeSpark().get(), Constants.IntakeAndArmConstants.intakeSpeed, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Intake Forwards: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Intake Forwards, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Intake Forwards: OK \u2713");
                System.out.print("------------------------------");
            }
            //stop intake to prepare for next test
            intake.stop();
        }

        else if(timer.get() < 2) {
            intake.runIntake(Direction.BACKWARDS);
        }
        else if(timer.get() == 2) {
            if(!tolerance(intake.getIntakeSpark().get(), -Constants.IntakeAndArmConstants.intakeSpeed, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Intake Backwards: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Intake Backwards, ";
            }
            else {
                System.out.println("------------------------------");
                System.out.println("Intake Backwards: OK \u2713");
                System.out.println("------------------------------");
            }
            //stop intake to prepare for next test
            intake.stop();
        }

        else if(timer.get() < 3) {
            arm.runArm(ArmState.HIGH);
        }
        else if(timer.get() == 3) {
            if(!tolerance(arm.getArmSpark().get(), Constants.IntakeAndArmConstants.pidHighSetPoint, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Arm High/Up: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Arm High/Up, ";
            }
            else {
                System.out.println("------------------------------");
                System.out.println("Arm High/Up: OK \u2713");
                System.out.println("------------------------------");
            }
        }
        
        else if(timer.get() < 4) {
            arm.runArm(ArmState.LOW);
        }
        else if (timer.get() == 4) {
            if(!tolerance(arm.getArmSpark().get(), Constants.IntakeAndArmConstants.pidLowSetPoint, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Arm Low/Down: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Arm Low/Down, ";
            }
            else {
                System.out.println("------------------------------");
                System.out.println("Arm Low/Down: OK \u2713");
                System.out.println("------------------------------");
            }
        }

        else if(timer.get() < 5) {

        }

        //display all broken subsystems
        else if (timer.get() == 30) {//TODO: 30 is temporary
            System.out.println("Broken Commands/Subsystems: " + fail);
        }
    }

    public boolean tolerance(double value, double expectedValue, double tolerance) {
        return value <= expectedValue + tolerance && value >= expectedValue - tolerance;
    }
}
