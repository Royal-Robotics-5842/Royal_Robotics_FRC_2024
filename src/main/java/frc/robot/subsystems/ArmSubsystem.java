package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax ArmLeft = new CANSparkMax(31, MotorType.kBrushless);
    CANSparkMax ArmRight = new CANSparkMax(32, MotorType.kBrushless);

    public RelativeEncoder ArmEncoder = ArmLeft.getEncoder();

    SparkPIDController ArmController = ArmLeft.getPIDController();

    public double angle = 0.0;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(360, 360);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(); //Making Goal
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(); //Making Setpoint


    public ArmSubsystem(){
        ArmLeft.restoreFactoryDefaults();
        ArmRight.restoreFactoryDefaults();

        ArmLeft.setInverted(false);
        ArmRight.setInverted(true);

        ArmRight.follow(ArmLeft);

        ArmLeft.setSmartCurrentLimit(40);
        ArmRight.setSmartCurrentLimit(40);

        ArmController.setP(0.05);
        ArmController.setI(0);
        ArmController.setD(0.02);

        ArmRight.setIdleMode(IdleMode.kBrake);
        ArmLeft.setIdleMode(IdleMode.kBrake);
    }

    public void moveArm(int input){
        switch (input) {
            case 1:
                angle = 0.0;
                break;
            case 2:
                angle = -38;
                break;
            case 3:
                angle = -92;
                break;
            case 4:
                angle = -31.8;
                break;
        }

        m_goal = new TrapezoidProfile.State(angle, 0);
        var profile = new TrapezoidProfile(m_constraints);
        m_setpoint = profile.calculate(.02,m_setpoint,m_goal);
        double armSpeed = m_setpoint.position;

        System.out.println(ArmEncoder.getPosition());

        ArmController.setReference(armSpeed,ControlType.kPosition);
    }
}
