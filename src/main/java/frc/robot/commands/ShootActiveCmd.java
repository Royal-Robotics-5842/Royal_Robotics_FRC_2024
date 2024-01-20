package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootActiveCmd extends Command  {

    private ShootSubsystem shootSubsystem;
    private boolean active;

    public ShootActiveCmd(ShootSubsystem shootSubsystem, boolean avtive) {
        this.active = active;
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ShootActiceCmd started!");
    }

    @Override
    public void execute() {
        shootSubsystem.setPosition(active);
    }

    @Override
    public void end(boolean interupted) {
        System.out.println("ShootActiveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return !active;
    }

    

    
}
