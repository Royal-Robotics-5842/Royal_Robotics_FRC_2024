package frc.robot.commands.EndgameCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndgameSubsystem;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

public class EndgameRight extends Command {

    private final EndgameSubsystem m_endgameSubsystem;
    private double egspeedR;
    private boolean invert;
   

    public EndgameRight(EndgameSubsystem endgameSubsystem, double right) {
      egspeedR = right;
      m_endgameSubsystem = endgameSubsystem;
      addRequirements(endgameSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println("EndgameRigjtCmd.java started");
    }
  
    @Override
    public void execute() {
      if (RobotContainer.m_operatorController.a().getAsBoolean())
      {
        invert = true;
        m_endgameSubsystem.setMotors(0,-egspeedR);
      }

      else 
      {
        invert = false;
        m_endgameSubsystem.setMotors(0, egspeedR);
      }
      
    }
  
    @Override
    public void end(boolean interrupted) {
      m_endgameSubsystem.setMotors(0,0);
      System.out.println("EndgameRightCmd.java ended");
    }
  
    @Override
    public boolean isFinished() {
        if(((m_endgameSubsystem.getRightLimit() == false) && (invert == true)))
        {
            return true;
        }
        return false;
    }
}