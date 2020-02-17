/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p> It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kInvertMotor = -1.0;

  // An example trajectory to follow.  All units in meters.
  /*
  * rotation = 0
  * x = 5
  y = 0
  */

  // On-the-go route finding waypoint variables
  public static final int pointx = 0;
  public static final int pointy = 0;
  public static final int rotation = 0;
  public static final int waypointx = 0;
  public static final int waypointy = 0;

  // Drive Train motors
  public static final int kR1MotorPort = 0;
  public static final int kR2MotorPort = 1;
  public static final int kR3MotorPort = 2;
  public static final int kL1MotorPort = 3;
  public static final int kL2MotorPort = 4;
  public static final int kL3MotorPort = 5;

  // Shooter spinner and turret moving motors
  public static final int kShooterMotorRPort = 6;
  public static final int kShooterMotorLPort = 7;
  public static final int kTurretMotorPort = 8;

  // Climber motors
  public static final int kClimberMotor1Port = 9;
  public static final int kClimberMotor2Port = 10;

  // Power cell intake and feeder motors
  public static final int kIntakeMotorPort = 11;
  //public static final int kIndexerMotorPort = 12; TODO: Both of these two constants are unused
  //public static final int klTowerMotorPort = 13;
  public static final int kFeederMotor1Port = 14;
  public static final int kFeederMotor2Port = 15;

  // Sticks
  public static final int kRightStickPort = 0;
  public static final int kLeftStickPort = 1;
  public static final int kGamepadPort = 2;
  // TODO: Add a free button mapping for running the on-the-go pathfinder command
  
  // Gamepad POV values in degrees
  public static final int kPovUp = 0;
  public static final int kPovDown = 180;

  // Solenoids
  public static final int kIntakeSolenoidPort = 0;

  // Turret limit switches
  public static final int kLimitSwitchRPort = 15;
  public static final int kLimitSwitchLPort = 16;

  // Need to edit all the numbers under this for 2020
  public static final int[] kLeftEncoderPorts = new int[] { 2, 3 };
  public static final int[] kRightEncoderPorts = new int[] { 0, 1 };
  public static final boolean kLeftEncoderReversed = false;
  public static final boolean kRightEncoderReversed = false;

  public static final double kTrackwidthMeters = 0.77;

  public static final int kEncoderCPR = 128;
  public static final double kWheelDiameterMeters = 0.1;
  public static final double kEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

  public static final boolean kGyroReversed = true;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or
  // theoretically for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining
  // these values for your robot.
  public static final double ksVolts = 1.43;
  public static final double kvVoltSecondsPerMeter = 5.15;
  public static final double kaVoltSecondsSquaredPerMeter = 0.384;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = -12.5;

  public static final int kDriverControllerPort = 1;

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
}
