package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShootSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor1 = new CANSparkMax(0, MotorType.kBrushless); 
    private CANSparkMax shootMotor2 = new CANSparkMax(1, MotorType.kBrushless);
    
    public ShootSubsystem() {    
    }
    public void setPosition(boolean active) {
        if (active) {
            shootMotor1.set(.5);
            shootMotor2.set(0.5);
        } else {
            shootMotor1.set(0);
            shootMotor2.set(0);
        }
    }
     
}
