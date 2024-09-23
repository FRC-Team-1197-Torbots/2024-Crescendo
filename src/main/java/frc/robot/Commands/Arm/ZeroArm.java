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
        m_Arm.setZeroingArm(false);
        m_Arm.runArm(-0.8);
    }

    @Override
    public void end(boolean isFinished) {
        m_Arm.setZeroingArm(true);
        m_Arm.stopMotor();
        m_Arm.resetArm();
    }

    @Override
    public boolean isFinished() {
        return m_Arm.getArmOutput() > 14;
    }
}
