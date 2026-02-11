package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;

public class Joystick extends GenericHID {
    public Joystick(int port) {
        super(port);
    }
}
