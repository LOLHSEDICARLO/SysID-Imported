// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


//TODO:  Use Config for TalonFX motors, set the inverted in config (see depricated setInverted below near end of this class)
//set right motor to inverted, using TalonFXConfiguration object, in MotorOutputConfigs group).
//see 2025-Robot Elevator subsystem for example

public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.
    private final TalonFX m_leftMotor = new TalonFX(DriveConstants.kLeftMotor1Port, "usb");
    private final TalonFX m_leftFollower = new TalonFX(DriveConstants.kLeftMotor2Port, "usb");

  // The motors on the right side of the drive.Left
  private final TalonFX m_rightMotor = new TalonFX(DriveConstants.kRightMotor1Port, "usb");
  private final TalonFX m_rightFollower = new TalonFX(DriveConstants.kRightMotor2Port, "usb");

  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

// The left-side drive encoder 
//CANcoders are absolute encoders used to determine initial offset of angle motor only
//The relative encoder below  is an external relative encoder using 2 ports, for chA and chB,
//and uses the WPILib "Encoder" class - requires 2 channels, plus direction
//TODO - use the relative encoder internal to the TalonFX 
//for now, use the internal TalonFX encoder (sensor) - no need to use encoders below

  // The left-side drive encoder (external encoder with 2 channels)
  private final Encoder m_leftEncoder =
      new Encoder(
        DriveConstants.kLeftEncoderPorts[0],
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderReversed);  //TODO- check if reversed
  

  // The right-side drive encoder (external encoder with 2 channels)
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed); //TODO - check if reversed

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // private final MutVoltage m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
 private final MutDistance m_distance = Meters.mutable(0);
 // private final MutDistance m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  //private final MutLinearVelocity m_velocity = mutable(MetersPerSecond.of(0));

//Create a method that gets distance from Postion - see format from other classes
 // DriveConstants.kEncoderDistancePerRevolution)*(m_leftMotor.getPosition())
 //Or just use Position (and Velocity) scale, without Mute Replace??

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Voltage volts) -> {
                m_leftMotor.setVoltage(volts.in(Volts));
                m_rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> { //TODO: SWITCH from getDistance to motor.getPosition (scaled), getRate to motor.getVelocity (scaled)
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                  ///getPosition returns Rotations. Need to multiply by distance per revolutions to get distance (meters)
                   // .linearPosition(m_distance.mut_replace(m_leftMotor.getPosition().getValueAsDouble()*DriveConstants.kEncoderDistancePerRevolution, Meters))
                  .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters)) //distance since last reset, as scaled by setDistancePerPuls
                   
                  ///getVelocity returns rotations/sec. Need to multiply by distance per revolutions to get meters/sec
                 //.linearVelocity(m_velocity.mut_replace(m_leftMotor.getVelocity().getValueAsDouble()*DriveConstants.kEncoderDistancePerRevolution, MetersPerSecond));
                  .linearVelocity(m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));  //distance per second, as scaled by the value of distance per pulse
                
                 // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                  
                ///getPosition returns Rotations. Need to multiply by distance per revolutions to get distance (meters)
                //  .linearPosition(m_distance.mut_replace(m_rightMotor.getPosition().getValue()*DriveConstants.kEncoderDistancePerRevolution, Meters))
                .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))  
                    
                ///getVelocity returns rotations/sec. Need to multiply by distance per revolutions to get meters/sec
                //  .linearVelocity(m_velocity.mut_replace(m_rightMotor.getVelocity().getValue()*DriveConstants.kEncoderDistancePerRevolution, MetersPerSecond));
                .linearVelocity(m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public Drive() {

    // Make the second motors (back motors) on each side of the drivetrain into followers
   m_rightFollower.setControl(new Follower(DriveConstants.kRightMotor1Port, false));
   m_leftFollower.setControl(new Follower(DriveConstants.kLeftMotor1Port, false));
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true); ///TODO - determine if this is true or not

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

}