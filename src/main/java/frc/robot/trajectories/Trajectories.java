package frc.robot.trajectories;

import java.io.IOException;
import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class Trajectories {

  // public Trajectories() {

  // }  //Max velocity & acceleration

  // public static CommandGroupBase followTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
      
  //     RamseteCommand command = new RamseteCommand(
  //     trajectory,
  //     drivetrain::getPose,
  //     new RamseteController(Constants.DriveTrainConstants.kRamseteB, Constants.DriveTrainConstants.kRamseteZeta),
  //     drivetrain.getFeedforward(),
  //     drivetrain.getKinematics(),
  //     drivetrain::getSpeeds,
  //     drivetrain.getLeftPIDController(),
  //     drivetrain.getRightPIDController(),
  //     drivetrain::setOutputVolts,
  //     drivetrain
  // );

  //   return command.andThen(() -> drivetrain.setOutputVolts(0, 0));
  // }
 
  // public static Trajectory loadTrajectory(String path) { //JSON path
  //   TrajectoryConfig config = new TrajectoryConfig(Constants.DriveTrainConstants.kMaxSpeedMetersPerSecond, Constants.DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared);
  //   config.setKinematics(Constants.DriveTrainConstants.kDriveKinematics);

  //   try {
  //     return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path));
  //   } catch (IOException e) {
  //     DriverStation.reportError("Trajectory not found. " + path, e.getStackTrace());
  //     e.printStackTrace();
  //     return TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d()), config);
  //     }
  //   }
 }