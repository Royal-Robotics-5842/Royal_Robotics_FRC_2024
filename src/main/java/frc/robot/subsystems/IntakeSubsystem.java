// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  
    public CANSparkMax Motor = new CANSparkMax(5, MotorType.kBrushless);
    
    /** Creates a new IntakeWithA. */
  public IntakeSubsystem() {

    Motor.restoreFactoryDefaults();
    
    Motor.setInverted(false);
    Motor.setSmartCurrentLimit(0);
    
    

    Motor.setIdleMode(IdleMode.kCoast);
    

  }

public void setMotors(double speed) {
  Motor.set(speed);
  
}

}