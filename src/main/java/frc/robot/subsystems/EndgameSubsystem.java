package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class EndgameSubsystem extends SubsystemBase 
{
    public final CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
    public final CANSparkMax rightMotor = new CANSparkMax(14, MotorType.kBrushless);

    public EndgameSubsystem() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {

    }

    public void setMotors(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    public double getSetTrigAng() {
        
    }
}
