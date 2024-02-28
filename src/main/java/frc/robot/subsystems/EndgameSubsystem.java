package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class EndgameSubsystem extends SubsystemBase 
{
    public final CANSparkMax leftMotor = new CANSparkMax(Constants.endgameConstants.leftCANID, MotorType.kBrushless);
    public final CANSparkMax rightMotor = new CANSparkMax(Constants.endgameConstants.rightCANID, MotorType.kBrushless);

    DigitalInput leftLimitSwitch = new DigitalInput(9);
    DigitalInput rightLimitSwitch = new DigitalInput(8);

    public EndgameSubsystem() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        rightMotor.setInverted(true);
        leftMotor.setInverted(true);

    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    public boolean getLeftLimit()
    {
        return leftLimitSwitch.get();
    }

    public boolean getRightLimit()
    {
        return rightLimitSwitch.get();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
       SmartDashboard.putBoolean("LeftLimit",leftLimitSwitch.get());
       SmartDashboard.putBoolean("RightLimit",rightLimitSwitch.get());
       SmartDashboard.putNumber("HIII", rightMotor.getAppliedOutput());
       
 } 
}