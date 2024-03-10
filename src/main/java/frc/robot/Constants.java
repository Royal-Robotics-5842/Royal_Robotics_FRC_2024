// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //https://github.com/SeanSun6814/FRC0ToAutonomous/blob/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot/Constants.java

  public static final class ModuleConstants
  {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //https://www.swervedrivespecialties.com/products/billet-wheel-4d-x-1-5w-bearing-bore
    public static final double kDriveMotorGearRatio = 6.75; //https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    public static final double kTurinigMotorGearRatio = 150/7; //https://www.andymark.com/products/mk4i-swerve-modules#:~:text=The%20full%20steering%20ratio%20of,1%20and%20Falcon%20500%20motors.
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio*Math.PI*kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurinigMotorGearRatio*2*Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kTurnP = 0.0073; //Krish Updated
  }

  public static final class DriveConstants
  {
    public static final double kTrackWidth = Units.inchesToMeters(28);
        // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(28);
        // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //FR
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //FL
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //BR
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //BL

      public static final int kFrontLeftDriveMotorPort = 23;
      public static final int kBackLeftDriveMotorPort = 25;
      public static final int kFrontRightDriveMotorPort = 24;
      public static final int kBackRightDriveMotorPort = 22;

      public static final int kFrontLeftTurningMotorPort = 13;
      public static final int kBackLeftTurningMotorPort = 15;
      public static final int kFrontRightTurningMotorPort = 14;
      public static final int kBackRightTurningMotorPort = 12;

      public static final boolean kFrontLeftTurningMotorReversed = true;
      public static final boolean kBackLeftTurningMotorReversed = true;
      public static final boolean kFrontRightTurningMotorReversed = true; 
      public static final boolean kBackRightTurningMotorReversed = true;

      public static final boolean kFrontLeftDriveMotorReversed = false;
      public static final boolean kBackLeftDriveMotorReversed = false;
      public static final boolean kFrontRightDriveMotorReversed = false;
      public static final boolean kBackRightDriveMotorReversed = true;

      public static final int kFrontLeftTurnAbsoluteEncoderPort = 33;
      public static final int kBackLeftTurnAbsoluteEncoderPort = 35;
      public static final int kFrontRightTurnAbsoluteEncoderPort = 34;
      public static final int kBackRightTurnAbsoluteEncoderPort = 32;

      public static final double kPhysicalMaxSpeedMetersPerSecond = 4; //Units.feetToMeters(14.5); //https://www.andymark.com/products/mk4i-swerve-modules#:~:text=The%20full%20steering%20ratio%20of,1%20and%20Falcon%20500%20motors.
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI; //Value is based on above value from NEO
      
      //public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4; //Dont want to run all swerve at max speed, dividing by 4
      //public static final double kTeleDrive MaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4 ; //Dont want to run all swerve at max speed, dividing by 4

      //public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //Value 3 is not used
      //public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; //Value 3 is not used
  }

  public static final class AutoConstants 
  {  
      public static final double AutonMultiplier = 0.5; //Run auton at set speed compared to tele 

      public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * AutonMultiplier ;
      public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * AutonMultiplier;
      //public static final double kMaxAccelerationMetersPerSecondSquared = 2;
      //public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 1;//Math.PI / 4;
      public static final double kPXController =3.5; //Value is arbitrary
      public static final double kPThetaController = 1.5; //Value is arbitrary

  }


  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 2;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    
    public static final double kDeadband = 0.05;
  }

  public static final class armConstants 
  {
    public static double ArmIntake = 0;
    public static double ArmShotSpeaker = 41;
    public static double ArmAmp = 56;

    public static int leftCANID = 41; 
    public static int rightCANID = 42;
  }


public static final class shooterConstants 
  {
    public static int leftCANID = 61; 
    public static int rightCANID = 62;
  }

}
