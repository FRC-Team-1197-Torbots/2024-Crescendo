package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;

public class AutoArm extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm m_Arm;
    private ArmStates m_ArmStates;
    
    public AutoArm(Arm subsystem, ArmStates armStates) {
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
            //System.out.println("Finished moving arm");
            //m_Arm.setStates(ArmStates.STORE);
            SmartDashboard.putBoolean("Arm on Target", m_Arm.onTarget());
        
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Arm.autoOnTarget();
    /* 
    if(!m_BreakBeam.get() && m_ArmStates == ArmStates.INTAKE){
        return true;
    }else{
        return false;
    }*/
  }

//limelight procedure
//1. Have driver select with button what to target: amp, speaker or source  
//2. Based on alliance and target, pick apriltag id number
//3. Look for the apriltag, and retrieve the distance and maybe angle

}
