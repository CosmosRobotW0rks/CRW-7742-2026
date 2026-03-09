package frc.robot.subsystems.neopixel;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NeoPixelSubsystem extends SubsystemBase {
    private final AddressableLED mLed; 
    private final AddressableLEDBuffer mLedBuffer;

    public enum NeoPixelCondition{
        ENABLED,
        PREPARING_SHOOT,

        IN_SHOOTING_DISTANCE,
        SHOOTING
    }

    Map<NeoPixelCondition, Boolean> condMap = new HashMap<NeoPixelSubsystem.NeoPixelCondition,Boolean>();

    public NeoPixelSubsystem()
    {
        mLed = new AddressableLED(9);
        mLedBuffer = new AddressableLEDBuffer(18);
        mLed.setLength(mLedBuffer.getLength());
        mLed.setData(mLedBuffer);
        mLed.start();

        for(NeoPixelCondition cond : NeoPixelCondition.values())
        {
            condMap.put(cond,false);
        }
    }

    public void setConditionState(NeoPixelCondition cond, boolean state)
    {
        condMap.put(cond, state);
    }

    private boolean getCondState(NeoPixelCondition cond)
    {
        return condMap.get(cond);
    }



    @Override
    public void periodic() {
        condMap.put(NeoPixelCondition.ENABLED, DriverStation.isEnabled());


        // Cond checkers

        if(getCondState(NeoPixelCondition.ENABLED)) LEDPattern.solid(Color.kAqua).applyTo(mLedBuffer);
        else LEDPattern.solid(Color.kRed).applyTo(mLedBuffer);
        
        if(getCondState(NeoPixelCondition.PREPARING_SHOOT)) 
            LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kLightBlue)
            .scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(10, MetersPerSecond), Distance.ofBaseUnits(1, Meter))
            .applyTo(mLedBuffer);

        if(getCondState(NeoPixelCondition.SHOOTING))
            LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kDarkGreen)
            .scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(10, MetersPerSecond), Distance.ofBaseUnits(1, Meter))
            .applyTo(mLedBuffer);

        mLed.setData(mLedBuffer);
    }
}
