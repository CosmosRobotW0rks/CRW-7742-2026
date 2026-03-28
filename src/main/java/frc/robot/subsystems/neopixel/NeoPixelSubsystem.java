package frc.robot.subsystems.neopixel;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;

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

    public enum NeoPixelCondition {
        ENABLED,
        PREPARING_SHOOT,

        IN_SHOOTING_DISTANCE,
        SHOOTING,

        ACTIVELY_READING_TAGS
    }

    Map<NeoPixelCondition, Boolean> condMap = new HashMap<NeoPixelSubsystem.NeoPixelCondition, Boolean>();

    public NeoPixelSubsystem() {
        mLed = new AddressableLED(9);
        mLedBuffer = new AddressableLEDBuffer(18);
        mLed.setLength(mLedBuffer.getLength());
        mLed.setData(mLedBuffer);
        mLed.start();

        for (NeoPixelCondition cond : NeoPixelCondition.values()) {
            condMap.put(cond, false);
        }
    }

    public void setConditionState(NeoPixelCondition cond, boolean state) {
        condMap.put(cond, state);
    }

    private boolean getCondState(NeoPixelCondition cond) {
        return condMap.get(cond);
    }

    @Override
    public void periodic() {

        condMap.put(NeoPixelCondition.ENABLED, DriverStation.isEnabled());

        boolean enabled = getCondState(NeoPixelCondition.ENABLED);
        boolean reading = getCondState(NeoPixelCondition.ACTIVELY_READING_TAGS);
        boolean preparing = getCondState(NeoPixelCondition.PREPARING_SHOOT);
        boolean shooting = getCondState(NeoPixelCondition.SHOOTING);

        if (!enabled) {
            LEDPattern.solid(Color.kRed).applyTo(mLedBuffer);
        }

        else if (shooting) {

            LEDPattern stripePattern;

            if (reading) {
                stripePattern = LEDPattern.steps(Map.of(
                        0,   Color.kGreen,
                        0.1, Color.kBlue,
                        0.3, Color.kGreen,
                        0.4, Color.kBlue,
                        0.6, Color.kGreen,
                        0.7, Color.kBlue
                        ));
            } else {
                stripePattern = LEDPattern.steps(Map.of(
                        0,   Color.kRed,
                        0.1, Color.kBlue,
                        0.3, Color.kRed,
                        0.4, Color.kBlue,
                        0.6, Color.kRed,
                        0.7, Color.kBlue));
            }

            stripePattern
                    .scrollAtAbsoluteSpeed(
                            LinearVelocity.ofBaseUnits(25, MetersPerSecond),
                            Distance.ofBaseUnits(1, Meter))
                    .applyTo(mLedBuffer);
        }
        else if (preparing) {

            if (reading) {
                LEDPattern.steps(Map.of(
                        0,   Color.kGreen,
                        0.1, Color.kBlue,
                        0.3, Color.kGreen,
                        0.4, Color.kBlue,
                        0.6, Color.kGreen,
                        0.7, Color.kBlue)).applyTo(mLedBuffer);
            } else {
                LEDPattern.steps(Map.of(
                        0,   Color.kRed,
                        0.1, Color.kBlue,
                        0.3, Color.kRed,
                        0.4, Color.kBlue,
                        0.6, Color.kRed,
                        0.7, Color.kBlue)).applyTo(mLedBuffer);
            }
        }

        else {

            if (reading) {
                LEDPattern.solid(Color.kGreen).applyTo(mLedBuffer);
            } else {
                LEDPattern.solid(Color.kLightBlue).applyTo(mLedBuffer);
            }
        }

        mLed.setData(mLedBuffer);
    }
}
