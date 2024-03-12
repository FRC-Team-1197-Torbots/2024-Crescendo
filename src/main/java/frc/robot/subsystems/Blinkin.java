package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class Blinkin extends SubsystemBase{

    private final Spark m_Blinkin;
    private final Intake m_Intake;

    public Blinkin(Intake intake) {
        m_Blinkin = new Spark(BlinkinConstants.Blinkin);
        m_Intake = intake;
    }

    public void setColor(double value) {
        m_Blinkin.set(value);
    }
}
