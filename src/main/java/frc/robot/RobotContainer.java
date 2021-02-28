// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Create the controllers
  GenericHID driverController = new XboxController(Constants.DRIVE_CONTROLLER);

  // Driver xBox Buttons

  Button A_Button = new JoystickButton(driverController, 1);
  Button B_Button = new JoystickButton(driverController, 2);
  Button X_Button = new JoystickButton(driverController, 3);
  Button Y_Button = new JoystickButton(driverController, 4);
  Button LB_Button = new JoystickButton(driverController, 5);
  Button RB_Button = new JoystickButton(driverController, 6);
  Button Select_Button = new JoystickButton(driverController, 7);
  Button Start_Button = new JoystickButton(driverController, 8);
  Button Left_Stick_Button = new JoystickButton(driverController, 9);
  Button Right_Stick_Button = new JoystickButton(driverController, 10);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Prevents robot from going faster than it is capable of achieving with the given voltage supply
    var autoVoltageContraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward( Constants.ksVolts, 
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);
    //Wraps together path constraints
    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                           Constants.kMaxAccelerationMetersPerSecondSquared)
                           
                           .setKinematics(Constants.kDriveKinematics)

                           .addConstraint(autoVoltageContraint);

    //This can be replaced with a PathWeaver JSON
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      //Start at the origin facing the +x direction
      new Pose2d(0, 0, new Rotation2d(0)),
      //Pass through these two interior waypoints, making an 's' curve path
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      //End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      //Pass config
      config
    );


    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      m_driveSubsystem::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts, 
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      m_driveSubsystem::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      //RamseteCommand passes volts ot the callback
      m_driveSubsystem::tankDriveVolts,
      m_driveSubsystem
    
    );
    //Ensure robot's location on the coordinate system and the trajectory's starting position are the same
    //Reset odometry to the starting pose of the trajectory
    m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());



    return ramseteCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
  }
}
