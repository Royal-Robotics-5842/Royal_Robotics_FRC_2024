package frc.robot.commands.EndgameCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndgameSubsystem;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

public class EndgameLeft extends Command {

    private final EndgameSubsystem m_endgameSubsystem;
    private double egspeedL;
    private boolean invert;
   

    public EndgameLeft(EndgameSubsystem endgameSubsystem, double left) {
      egspeedL = left;
      this.invert = invert;
      m_endgameSubsystem = endgameSubsystem;
      addRequirements(endgameSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println("EndgameLeftCmd.java started");
    }
  
    @Override
    public void execute() {
      if (RobotContainer.m_operatorController.a().getAsBoolean())
      {
        invert = true;
        m_endgameSubsystem.setMotors(-egspeedL, 0);
      }

      else 
      {
        invert = false;
        m_endgameSubsystem.setMotors(egspeedL, 0);
      }
      
    }
  
    @Override
    public void end(boolean interrupted) {
      m_endgameSubsystem.setMotors(0,0);
      System.out.println("EndgameLeftCmd.java ended");
    }
  
    @Override
    public boolean isFinished() {
        if(((m_endgameSubsystem.getLeftLimit() == false) && (invert == true)))
        {
            return true;
        }
        return false;
    }
}