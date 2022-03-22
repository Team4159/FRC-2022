package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.PowerOutput;

public class Drive extends CommandBase{

  private Drivetrain drivetrain;
  private Joystick leftJoystick;
  private Joystick rightJoystick;
  private PowerOutput powerOutput;

  public Drive(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick, PowerOutput powerOutput) {
    this.drivetrain = drivetrain;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.powerOutput = powerOutput;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (powerOutput==PowerOutput.FULL_POWER){
      drivetrain.drive(leftJoystick.getY(), rightJoystick.getY());
    } else {
      drivetrain.drive(leftJoystick.getY()*0.5, rightJoystick.getY()*0.5);
    }
  }
  
}
