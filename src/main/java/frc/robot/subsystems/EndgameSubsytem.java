package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class EndgameSubsytem extends SubsystemBase 
{
    public final CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
    public final CANSparkMax rightMotor = new CANSparkMax(14, MotorType.kBrushless);

    public EndgameSubsytem() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        /*
        public void setMotors(double leftSpeed, double rightSpeed) {
            leftMotor.set(leftSpeed);
            rightMotor.set(rightSpeed);
        }
        */
    }
}
