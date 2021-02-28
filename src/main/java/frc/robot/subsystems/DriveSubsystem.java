// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftEncoder.setVelocityConversionFactor(Constants.ENCODER_LOW_METERS_PER_SECOND);
    m_rightEncoder.setVelocityConversionFactor(Constants.ENCODER_LOW_METERS_PER_SECOND);

    m_leftEncoder.setPositionConversionFactor(Constants.ENCODER_LOW_METERS);
    m_rightEncoder.setPositionConversionFactor(Constants.ENCODER_LOW_METERS);
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    resetEncoders();
  }



  public CANSparkMax frontLeft = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_CAN, MotorType.kBrushless);
  public CANSparkMax frontRight = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_CAN, MotorType.kBrushless);
  public CANSparkMax backLeft = new CANSparkMax(Constants.BACK_LEFT_MOTOR_CAN, MotorType.kBrushless);
  public CANSparkMax backRight = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_CAN, MotorType.kBrushless);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  //Group motors

  public SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeft, backLeft);
  public SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRight, backRight);
   
  //Defining Differential Drive

  public DifferentialDrive differentialRocketLeagueDrive = new DifferentialDrive(leftGroup, rightGroup);

  //Characterization - Pathweaver

  //Feeforward/Feedback Gains

    //Moved to Constants.java

  //DifferentialDriveKinematics

    //Moved to Constants.java

  //Max Trajectory Velocity/Acceleration
  //Max velocity set shomewhat below nominal free-speed of the robot
  //Due to the later use of `DifferentialDriveVoltageConstraint` the max acceleration value is not crucial

      //Moved to Constants.java

  //Ramesete Parameters
  //Parameters for Ramsete contorller.  Values are what are used for most robots

    //Moved to Constants.java

  //Create encoder from Spark MAX

  public CANEncoder m_leftEncoder = frontLeft.getEncoder();
  public CANEncoder m_rightEncoder = frontRight.getEncoder();

  //Gyro

  private final Gyro m_gyro = new ADXRS450_Gyro();

  //Odometry

  private final DifferentialDriveOdometry m_odometry; //Used to find position relative to starting location

  //Encoder conversion from rotations to meters
  //Encoder Accessor Method

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    m_leftEncoder.setVelocityConversionFactor(Constants.ENCODER_LOW_METERS_PER_SECOND);
    m_rightEncoder.setVelocityConversionFactor(Constants.ENCODER_LOW_METERS_PER_SECOND);
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public double getHeading(){
    return m_gyro.getRotation2d().getDegrees();
  }


  public Pose2d getPose(){
    return m_odometry.getPoseMeters();  //Returns the position of the robot on the field, meters
  }

  //Control the differential drive with voltage
  public void tankDriveVolts(double leftVolts, double rightvolts){
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightvolts);
    differentialRocketLeagueDrive.feed();
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }
}
