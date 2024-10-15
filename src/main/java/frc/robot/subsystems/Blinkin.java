package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class Blinkin extends SubsystemBase{

    private final Spark m_Blinkin;

    public Blinkin() {
        m_Blinkin = new Spark(BlinkinConstants.Blinkin);
    }

    public void setColor(double value) {
        m_Blinkin.set(value);
    }
}
