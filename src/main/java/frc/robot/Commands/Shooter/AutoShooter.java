package frc.robot.Commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Intake.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ArmConstants.ArmStates;

public class AutoShooter extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_Shooter;
    private final Arm m_Arm;
    private ArmStates m_ArmStates;
    private PIDController m_ShooterPID;
    private double targetRPM = -4000;
    private boolean ShootFinished = false;

    public AutoShooter(Shooter subsystem, Arm armsubsystem, ArmStates armstates) {
        m_Shooter = subsystem;
        m_Arm = armsubsystem;
        m_ArmStates = armstates;
        m_ShooterPID = new PIDController(m_Shooter.getKp(), 0.000001, 0);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem, armsubsystem);
      }

      @Override
      public void initialize() {
        ShootFinished = false;
        m_Arm.setStates(m_ArmStates);

        super.initialize();
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        super.execute();
        SmartDashboard.putBoolean("Shooter finished", ShootFinished);
        //System.out.println("Going up");
        //System.out.println("revving");
        m_Shooter.runShooter(0.85);
        if(m_Shooter.onTarget()){
          m_Shooter.runIntakeShooter();
        }
        //m_Shooter.runShooter(0.85);
      }

      @Override
      public void end(boolean interrupted) {    
          ShootFinished = true;
          SmartDashboard.putBoolean("Shooter finished", ShootFinished);
          //System.out.println("finished with shoot");
          m_Shooter.stopMotor();
          m_Shooter.stopIntakeShooter();

          super.end(interrupted);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            //super.isFinished();
            return !m_Shooter.getBreakBeamState();
            // return true;
        }
    
}
