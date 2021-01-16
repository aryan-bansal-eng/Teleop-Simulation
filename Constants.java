// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N7;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum RobotState {Disabled, Auton, Teleop};

  public static final int LEFT_MOTOR_PORT = 0, RIGHT_MOTOR_PORT = 1;

  //2 Falcon 500 motors on each side of the drivetrain.
  public static final DCMotor motors = DCMotor.getFalcon500(2);

  //Gearing reduction of the robot
  public static final double GEAR_REDUCTION = 10.0D;

  //Moment of Inertia of the robot
  public static final double MOMENT_OF_INERTIA = 7.469D;

  //Mass of the robot in kilograms
  public static final double ROBOT_MASS = 68.0388554D;

  //Radius of the wheels in meters
  public static final double WHEEL_RADIUS = Units.inchesToMeters(3);

  //Track width of the wheels(distance between left and right wheels)
  public static final double TRACK_WIDTH = Units.inchesToMeters(20);

  public static final Encoder leftEncoder = new Encoder(0, 1);
  public static final Encoder rightEncoder = new Encoder(2, 3);
  
  public static  final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  public static final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  
  public static final PWMTalonFX leftMotor = new PWMTalonFX(Constants.LEFT_MOTOR_PORT);
  public static final PWMTalonFX rightMotor = new PWMTalonFX(Constants.RIGHT_MOTOR_PORT);

  //The standard deviations for measurement noise: 
  //x and y: 0.001 m
  //heading: 0.001 rad
  //l and r velocity: 0.1 m/s
  //l and r position: 0.005 m
  public static final Vector<N7> STANDARD_DEVIATIONS = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

  public static final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(motors, GEAR_REDUCTION, MOMENT_OF_INERTIA, ROBOT_MASS, WHEEL_RADIUS, TRACK_WIDTH, STANDARD_DEVIATIONS);

  static
  {
    leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.WHEEL_RADIUS / 4096);
    rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.WHEEL_RADIUS / 4096);
  }
}
