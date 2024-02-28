package frc.robot.commands.EndgameCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndgameSubsystem;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

public class EndgameCmd extends Command {

    private final EndgameSubsystem m_endgameSubsystem;
    private double egspeedL;
    private double egspeedR;
    private boolean invert;
   

    public EndgameCmd(EndgameSubsystem endgameSubsystem, double left, double right, boolean invert) {
      egspeedL = left;
      egspeedR = right;
      this.invert = invert;
      m_endgameSubsystem = endgameSubsystem;
      addRequirements(endgameSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println("EndgameCmd.java started");
    }
  
    @Override
    public void execute() {
      if (RobotContainer.m_operatorController.a().getAsBoolean())
      {
        m_endgameSubsystem.setMotors(-egspeedL, -egspeedR);
      }

      else 
      {
        m_endgameSubsystem.setMotors(egspeedL, egspeedR);
      }
      
    }
  
    @Override
    public void end(boolean interrupted) {
      m_endgameSubsystem.setMotors(0,0);
      System.out.println("EndgameCmd.java ended");
    }
  
    @Override
    public boolean isFinished() {
        if(((m_endgameSubsystem.getLeftLimit() == false) && (m_endgameSubsystem.leftMotor.getAppliedOutput() < 0)) || ((m_endgameSubsystem.getRightLimit() == false) && (m_endgameSubsystem.rightMotor.getAppliedOutput() < 0)))
        {
            return true;
        }
        return false;
    }
}