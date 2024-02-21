package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.Arm;

public class RunArm extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm m_Arm;
    private ArmStates m_ArmStates;
    public RunArm(Arm subsystem, ArmStates armStates) {
        m_Arm = subsystem;
        m_ArmStates = armStates;
        //m_Speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {

        m_Arm.setStates(m_ArmStates);
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        //System.out.println("Going up");
      }

      @Override
        public void end(boolean interrupted) {
            m_Arm.setStates(ArmStates.STORE);
        
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
