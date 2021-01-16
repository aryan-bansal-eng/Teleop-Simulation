// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class OI {

    //Choose Keyboard 2 Joystick on simulation and use arrow keys to move
    private Joystick joy;

    public OI()
    {
        joy = new Joystick(0);
    }

    public double getThrottle()
    {
        return -joy.getRawAxis(1);
    }

    public double getTurn()
    {
        return joy.getRawAxis(0);
    }
}
