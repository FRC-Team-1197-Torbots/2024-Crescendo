package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class RunArm extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm m_arm;
    private double m_Speed;
    public RunArm(Arm subsystem, double speed) {
        m_arm = subsystem;
        m_Speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        //System.out.println("James' Arm Psuedocode Initialized");
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
        m_arm.runArm(m_Speed);
      }

      @Override
        public void end(boolean interrupted) {
            m_arm.stopMotor();
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

//limelight procedure
//1. Have driver select with button what to target: amp, speaker or source  
//2. Based on alliance and target, pick apriltag id number
//3. Look for the apriltag, and retrieve the distance and maybe angle

}
