package frc.robot.commands.ArmPositons;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

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

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    arm.moveArm(-30);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}


  // Returns true when the command should end.`
  @Override
  public boolean isFinished() {
    if (Math.abs(Math.abs(arm.ArmLeftEncoder.getPosition()) - Math.abs(arm.angle)) <= 0.5)
    {
      return true;
    }
    return false;
  }
}
