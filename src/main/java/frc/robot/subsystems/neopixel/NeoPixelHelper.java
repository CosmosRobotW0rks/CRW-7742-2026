package frc.robot.subsystems.neopixel;

public class NeoPixelHelper {
    private NeoPixelHelper(){}

    private static NeoPixelSubsystem subsys = null;
    
    public static void SetNeoPixelSubsystem(NeoPixelSubsystem neopixel)
    {
        subsys = neopixel;
    }
}
