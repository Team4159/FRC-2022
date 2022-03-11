package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase{

  private Drivetrain drivetrain;
  private Joystick leftJoystick;
  private Joystick rightJoystick;

  
  public Drive(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    this.drivetrain = drivetrain;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(-leftJoystick.getY(), rightJoystick.getY());
  }
  
}
