package frc.robot.trajectories;

import java.io.IOException;
import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class Trajectories {
 //Max velocity & acceleration
    public static CommandGroupBase getCommand(Drivetrain drivetrain, Trajectory trajectory){
        //double maxVoltage = 8;
        //DifferentialDriveVoltageConstraint constraint = new DifferentialDriveVoltageConstraint(drivetrain.getMotorFeedForward(), drivetrain.getDifferentialDriveKinematics(), maxVoltage);
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(1)).setKinematics(drivetrain.getDifferentialDriveKinematics());
        
        RamseteCommand command = new RamseteCommand(
           trajectory,
           drivetrain::getPose,
           new RamseteController(.2d,.2d),
           drivetrain.getMotorFeedForward(),
           drivetrain.getDifferentialDriveKinematics(),
           drivetrain::getVelocities,
           drivetrain.getLeftPIDController(),
           drivetrain.getRightPIDController(),
           (leftVoltage, rightVoltage) -> drivetrain.drive(leftVoltage/12, rightVoltage/12),
           drivetrain
        );
        return command.andThen(drivetrain::stop);
    }

    public static Trajectory loadTrajectory(String path) { //JSON path
        try {
          return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("output/" + path));
        } catch (IOException e) {
          DriverStation.reportError("Trajectory not found. " + path, e.getStackTrace());
          e.printStackTrace();
          return TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d()), new TrajectoryConfig(0, 0));
        }
      }
    
}
