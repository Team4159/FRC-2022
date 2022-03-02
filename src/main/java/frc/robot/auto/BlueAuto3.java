package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ThirdAuto extends CommandBase{
    package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Direction;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.Shoot;
import frc.robot.commands.CommandGroups.ArmAndIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.Trajectories;

//Does Nothing
public class BlueAuto3 extends CommandBase{
Drivetrain drivetrain;
Arm arm;
Intake intake;
Feeder feeder;
Shooter shooter;
Trajectories trajectories = new Trajectories();
Trajectory trajectory;

    public void BlueAuto3(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        trajectory =  trajectories.loadTrajectory("..trajectories\output\BlueAuto3.wpilib.json");
        new ParallelCommandGroup( //starting position and shoot
            Trajectories.followTrajectory(drivetrain, trajectory),
                new WaitCommand(1), //pause path run while shooting
                new MoveArm(arm, ArmState.LOW), //lower arm
                new RunFeeder(feeder, Direction.FORWARDS), //suck in ball
                new MoveArm(arm, ArmState.HIGH), //raise arm
                new RunNeck(neck, Direction.FORWARDS), //suck in
                new Shoot(shooter)
            ).withTimeout(1), //total: 2s
                
            new SequentialCommandGroup( //take ball 1
                new WaitCommand(1), 
                new MoveArm(arm, ArmState.LOW),
                new RunFeeder(feeder, Direction.FORWARDS), 
                new MoveArm(arm, ArmState.HIGH),
                new RunNeck(neck, Direction.FORWARDS) 
            ).withTimeout(2), //total: 5s, 2 imagines 2 second to first ball

            new SequentialCommandGroup( //take ball 2
                new WaitCommand(1), 
                new MoveArm(arm, ArmState.LOW), 
                new RunFeeder(feeder, Direction.FORWARDS), 
                new MoveArm(arm, ArmState.HIGH), 
                new RunNeck(neck, Direction.FORWARDS) 
            ).withTimeout(3), //total: 9s, 3 imagines 3 second to second ball

            new WaitCommand(1),
            new Shoot(shooter).withTimeout(1), //shoots both(both assumed) balls queued up
            //total: 10s

            new SequentialCommandGroup( //take ball 3
                new WaitCommand(1), 
                new MoveArm(arm, ArmState.LOW), 
                new RunFeeder(feeder, Direction.FORWARDS), 
                new MoveArm(arm, ArmState.HIGH), 
                new RunNeck(neck, Direction.FORWARDS)  
            ).withTimeout(2), //total: 13s

            new SequentialCommandGroup(
                new WaitCommand(1), 
                new MoveArm(arm, ArmState.LOW), 
                new RunFeeder(feeder, Direction.FORWARDS), 
                new MoveArm(arm, ArmState.HIGH), 
                new RunNeck(neck, Direction.FORWARDS)  
            ).withTimeout(2), //total: 15s ugh im gonna go overboard whatever lazy

            new WaitCommand(1),
            new Shoot(shooter).withTimeout(1) //last shoot, end auto. auto path def needs some work lmao                
            )  
        ).withTimeout(15); //< what ever is the purpose of this
    }
}


