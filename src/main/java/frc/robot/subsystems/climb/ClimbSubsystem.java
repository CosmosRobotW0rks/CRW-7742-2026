package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    public enum ClimbState{
        UP, 
        DOWN
    }

    ClimbState state = ClimbState.DOWN;

    Compressor compressor;
    Solenoid solenoid;

    public ClimbSubsystem()
    {
        compressor = new Compressor(ClimbConstants.PCMCANID, PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(ClimbConstants.PCMCANID, PneumaticsModuleType.CTREPCM, ClimbConstants.ValveChannel);

        compressor.disable();
        //compressor.enableDigital();

    }

    public Command Up()
    {
        return Commands.runOnce(() -> state = ClimbState.UP, this);
    }

    public Command Down()
    {
        return Commands.runOnce(() -> state = ClimbState.DOWN, this);
    }

    public Command Toggle()
    {
        return Commands.deferredProxy(() -> {
            if (state == ClimbState.UP)
                return Down();
            else
                return Up();
        });
    }


    @Override
    public void periodic() {
        switch (state) {
            case UP:
            solenoid.set(false);
            break;

            case DOWN:
            solenoid.set(true);
            break;

            default:
            break;
        }

        SmartDashboard.putBoolean("Climb/State", state == ClimbState.UP ? true : false);

    }
}
