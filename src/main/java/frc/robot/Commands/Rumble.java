package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends Command {

    private CommandXboxController m_Controller;
    private Timer timer;
    private double duration;

    public Rumble(CommandXboxController controller, double duration) {
        m_Controller = controller;
        this.duration = duration;
        timer = new Timer();
    }
    
    @Override
    public void initialize() {
        m_Controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.4);
        timer.restart();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        m_Controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > duration;
    }
    
}
