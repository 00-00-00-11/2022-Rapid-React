package frc.robot;




import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Joystick;
//  import frc.robot.commands.ExampleCommand;
//  import frc.robot.subsystems.ExampleSubsystem;
//  import edu.wpi.first.wpilibj2.command.Command;  







public class RobotContainer {
  public static  IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();  
  public static PS4Controller controller = new PS4Controller(0); 
   //public static Joystick controller = new Joystick(); 

  
    public static JoystickButton triangleButton = new JoystickButton(controller, 4);


  




  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {
    
    
    
    
    triangleButton.whileHeld(new IndexerCommand()); 
           //  XboxController controller = new XboxController()  
    
    
    

    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
