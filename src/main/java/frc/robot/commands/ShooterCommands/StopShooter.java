package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command  {

    private ShooterSubsystem shootSubsystem;
    public StopShooter(ShooterSubsystem shootSubsystem) {
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("StopShooter START");
    }

    @Override
    public void execute() {
        shootSubsystem.leftShooter.set(0);
        shootSubsystem.rightShooter.set(0);
    }

    @Override
    public void end(boolean interupted) {
        System.out.println("StopShooter START");
    }

    @Override
    public boolean isFinished() {
        return false;//(Math.abs(shootSubsystem.shooterEncoder.getVelocity()) - Math.abs(speed) >= 1);
    }

    

    
}
