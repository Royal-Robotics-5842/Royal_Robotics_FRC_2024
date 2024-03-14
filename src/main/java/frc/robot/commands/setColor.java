// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class setColor extends Command {
  /** Creates a new setColor
   *. */
  private final LEDSubsystem led;
  private String color;

  public setColor(LEDSubsystem led, String color) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.color = color;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if (color.equalsIgnoreCase("Red"))
    {
      for (var i = 0; i < led.m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        led.m_ledBuffer.setRGB(i, 255, 0, 0);
     }
     
     led.m_led.setData(led.m_ledBuffer);
    }  

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
}
}