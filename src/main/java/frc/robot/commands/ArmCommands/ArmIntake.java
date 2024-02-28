package frc.robot.commands.ArmCommands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmIntake extends Command {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubsystem arm;

  public ArmIntake(ArmSubsystem arm){
    this.arm = arm;
    addRequirements(arm);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Arm intake Start");
    arm.ArmRight.setIdleMode(IdleMode.kBrake);
    arm.ArmLeft.setIdleMode(IdleMode.kBrake);

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    arm.moveArm(Constants.armConstants.ArmIntake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    arm.setSpeed(0);
    arm.ArmRight.setIdleMode(IdleMode.kBrake);
    arm.ArmLeft.setIdleMode(IdleMode.kBrake);
  }


  // Returns true when the command should end.`
  @Override
  public boolean isFinished() {

    if((arm.getLimit() == false) && (arm.pidSpeed < 0.0))
    {
      return true;
    }

    if ((Math.abs(Math.abs(arm.ArmLeftEncoder.getPosition()) - Math.abs(Constants.armConstants.ArmIntake)) <= 0.5) )
    {
      System.out.println("Arm intake DONE");
      return true;
    }
    return false;
  }
}
