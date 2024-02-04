package frc.robot.commands.ArmPositons;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmWithController extends Command {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubsystem arm;
  private double speed;

  public ArmWithController(ArmSubsystem arm, double speed){
    this.arm = arm;
    this.speed = speed;
    addRequirements(arm);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
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
    arm.setSpeed(speed);
    SmartDashboard.putNumber("ARM ENCODER", arm.ArmEncoder.getPosition());
    SmartDashboard.putNumber("NCODER", RobotContainer.m_driverController.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}


  // Returns true when the command should end.`
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.m_operatorController.getLeftY()) <= 0.1);
  }
}