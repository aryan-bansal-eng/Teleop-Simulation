// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.TeleopCommand;

public class Teleop extends SubsystemBase {
  /** Creates a new Teleop. */
  public Teleop() {}

  public void initialize()
  {
    setDefaultCommand(new TeleopCommand());
  }

  public void move()
  {
    double throttle = Robot.getOI().getThrottle();
    double turn = Robot.getOI().getTurn();

    double left = 0.7 * throttle + 0.3 * turn;
    double right = 0.7 * throttle - 0.3 * turn;

    Constants.leftMotor.setVoltage(left);
    Constants.rightMotor.setVoltage(right);

    Constants.driveSim.setInputs(Constants.leftMotor.get() * RobotController.getInputVoltage(), Constants.rightMotor.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms.
    Constants.driveSim.update(0.02);

    // Update all of our sensors.
    Constants.leftEncoderSim.setDistance(Constants.driveSim.getLeftPositionMeters());
    Constants.leftEncoderSim.setRate(Constants.driveSim.getLeftVelocityMetersPerSecond());
    Constants.rightEncoderSim.setDistance(Constants.driveSim.getRightPositionMeters());
    Constants.rightEncoderSim.setRate(Constants.driveSim.getRightVelocityMetersPerSecond());

    //Distance generally does not mean displacement
    SmartDashboard.putNumber("Distance Left", Constants.leftEncoderSim.getDistance());
    SmartDashboard.putNumber("Distance Right", Constants.rightEncoderSim.getDistance());

    setDefaultCommand(new TeleopCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
