package frc.robot.Commands.Arm;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroArm extends Command {
    private final Arm m_Arm;

    public ZeroArm(Arm subsystem) {
        m_Arm = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_Arm.resettingArm(true);
    }

    @Override
    public void end(boolean isFinished) {
        m_Arm.resettingArm(false);
        m_Arm.stopMotor();
        m_Arm.resetArm();
    }

    @Override
    public boolean isFinished() {
        return m_Arm.getArmOutput() > 14;
    }
}
