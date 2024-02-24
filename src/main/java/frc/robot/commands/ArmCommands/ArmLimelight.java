package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class ArmLimelight extends Command {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubsystem arm;
  private final Limelight limelight = new Limelight(31, 18, 52);

  public ArmLimelight(ArmSubsystem arm){
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
  
    if(limelight.getLimelightX() == 0)
    {
      arm.moveArm(arm.ArmLeftEncoder.getPosition());
    }
    else
      arm.moveArm((90-limelight.angle()));
      System.out.println("Distance to Speaker: " +limelight.getDistanceFromLimelightToGoalInches());
      System.out.println("Angle to Speaker: " +(90 - limelight.angle()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}


  // Returns true when the command should end.`
  @Override
  public boolean isFinished() {
    return false;
  }
}