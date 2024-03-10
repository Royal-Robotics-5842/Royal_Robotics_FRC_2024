package frc.robot.commands.ArmCommands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmShotSpeaker extends Command {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubsystem arm;

  public ArmShotSpeaker(ArmSubsystem arm){
    this.arm = arm;
    addRequirements(arm);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Arm speaker START");
    arm.ArmRight.setIdleMode(IdleMode.kBrake);
    arm.ArmLeft.setIdleMode(IdleMode.kBrake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    arm.moveArm(Constants.armConstants.ArmShotSpeaker);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    System.out.println("Arm speaker DONE");
    arm.setSpeed(0);
    arm.ArmRight.setIdleMode(IdleMode.kBrake);
    arm.ArmLeft.setIdleMode(IdleMode.kBrake);
  }


  // Returns true when the command should end.`
  @Override
  public boolean isFinished() {

    if ((Math.abs(Math.abs(arm.ArmLeftEncoder.getPosition()) - Math.abs(Constants.armConstants.ArmShotSpeaker)) <= 0.5))
    {    
      return true;
    }
    return false;
  }
}
