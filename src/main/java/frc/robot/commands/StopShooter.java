package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class StopShooter extends Command  {

    private ShootSubsystem shootSubsystem;
    private double speed;
    public StopShooter(ShootSubsystem shootSubsystem) {
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shootSubsystem.leftShooter.set(0);
        shootSubsystem.rightShooter.set(0);
    }

    @Override
    public void end(boolean interupted) {
          
    }

    @Override
    public boolean isFinished() {
        return false;//(Math.abs(shootSubsystem.shooterEncoder.getVelocity()) - Math.abs(speed) >= 1);
    }

    

    
}
