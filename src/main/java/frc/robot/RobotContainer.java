/***
 *
 *                                        bbbbbbbb
 *    RRRRRRRRRRRRRRRRR                   b::::::b                                      tttt                       CCCCCCCCCCCCC                                            tttt                              iiii
 *    R::::::::::::::::R                  b::::::b                                   ttt:::t                    CCC::::::::::::C                                         ttt:::t                             i::::i
 *    R::::::RRRRRR:::::R                 b::::::b                                   t:::::t                  CC:::::::::::::::C                                         t:::::t                              iiii
 *    RR:::::R     R:::::R                 b:::::b                                   t:::::t                 C:::::CCCCCCCC::::C                                         t:::::t
 *      R::::R     R:::::R   ooooooooooo   b:::::bbbbbbbbb       ooooooooooo   ttttttt:::::ttttttt          C:::::C       CCCCCC   ooooooooooo   nnnn  nnnnnnnn    ttttttt:::::ttttttt      aaaaaaaaaaaaa   iiiiiiinnnn  nnnnnnnn        eeeeeeeeeeee    rrrrr   rrrrrrrrr
 *      R::::R     R:::::R oo:::::::::::oo b::::::::::::::bb   oo:::::::::::oo t:::::::::::::::::t         C:::::C               oo:::::::::::oo n:::nn::::::::nn  t:::::::::::::::::t      a::::::::::::a  i:::::in:::nn::::::::nn    ee::::::::::::ee  r::::rrr:::::::::r
 *      R::::RRRRRR:::::R o:::::::::::::::ob::::::::::::::::b o:::::::::::::::ot:::::::::::::::::t         C:::::C              o:::::::::::::::on::::::::::::::nn t:::::::::::::::::t      aaaaaaaaa:::::a  i::::in::::::::::::::nn  e::::::eeeee:::::eer:::::::::::::::::r
 *      R:::::::::::::RR  o:::::ooooo:::::ob:::::bbbbb:::::::bo:::::ooooo:::::otttttt:::::::tttttt         C:::::C              o:::::ooooo:::::onn:::::::::::::::ntttttt:::::::tttttt               a::::a  i::::inn:::::::::::::::ne::::::e     e:::::err::::::rrrrr::::::r
 *      R::::RRRRRR:::::R o::::o     o::::ob:::::b    b::::::bo::::o     o::::o      t:::::t               C:::::C              o::::o     o::::o  n:::::nnnn:::::n      t:::::t              aaaaaaa:::::a  i::::i  n:::::nnnn:::::ne:::::::eeeee::::::e r:::::r     r:::::r
 *      R::::R     R:::::Ro::::o     o::::ob:::::b     b:::::bo::::o     o::::o      t:::::t               C:::::C              o::::o     o::::o  n::::n    n::::n      t:::::t            aa::::::::::::a  i::::i  n::::n    n::::ne:::::::::::::::::e  r:::::r     rrrrrrr
 *      R::::R     R:::::Ro::::o     o::::ob:::::b     b:::::bo::::o     o::::o      t:::::t               C:::::C              o::::o     o::::o  n::::n    n::::n      t:::::t           a::::aaaa::::::a  i::::i  n::::n    n::::ne::::::eeeeeeeeeee   r:::::r
 *      R::::R     R:::::Ro::::o     o::::ob:::::b     b:::::bo::::o     o::::o      t:::::t    tttttt      C:::::C       CCCCCCo::::o     o::::o  n::::n    n::::n      t:::::t    tttttta::::a    a:::::a  i::::i  n::::n    n::::ne:::::::e            r:::::r
 *    RR:::::R     R:::::Ro:::::ooooo:::::ob:::::bbbbbb::::::bo:::::ooooo:::::o      t::::::tttt:::::t       C:::::CCCCCCCC::::Co:::::ooooo:::::o  n::::n    n::::n      t::::::tttt:::::ta::::a    a:::::a i::::::i n::::n    n::::ne::::::::e           r:::::r
 *    R::::::R     R:::::Ro:::::::::::::::ob::::::::::::::::b o:::::::::::::::o      tt::::::::::::::t        CC:::::::::::::::Co:::::::::::::::o  n::::n    n::::n      tt::::::::::::::ta:::::aaaa::::::a i::::::i n::::n    n::::n e::::::::eeeeeeee   r:::::r
 *    R::::::R     R:::::R oo:::::::::::oo b:::::::::::::::b   oo:::::::::::oo         tt:::::::::::tt          CCC::::::::::::C oo:::::::::::oo   n::::n    n::::n        tt:::::::::::tt a::::::::::aa:::ai::::::i n::::n    n::::n  ee:::::::::::::e   r:::::r
 *    RRRRRRRR     RRRRRRR   ooooooooooo   bbbbbbbbbbbbbbbb      ooooooooooo             ttttttttttt               CCCCCCCCCCCCC   ooooooooooo     nnnnnn    nnnnnn          ttttttttttt    aaaaaaaaaa  aaaaiiiiiiii nnnnnn    nnnnnn    eeeeeeeeeeeeee   rrrrrrr
 *
 *
 */

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  /* SUBSYSTEMS */
  public static DriveSubsystem m_driveSubsystem;
  public static ClimberSubsystem m_climberSubsystem;

  /* COMMANDS */
  public static ClimberCommand m_climberCommand;
  public static AutoClimberCommand m_autoClimberCommand;

  /* CONTROLLERS AND OTHER INPUTS */
  public static PS4Controller driverController;
  public static PS4Controller operatorController;

  Button driverElevatorButton =
      new JoystickButton(operatorController, PS4Controller.Button.kCircle.value);
  Button autoElevatorButton =
      new JoystickButton(operatorController, PS4Controller.Button.kCross.value);

  public RobotContainer() throws Exception {

    try {
      m_driveSubsystem = new DriveSubsystem();
    } catch (Exception err) {
      throw new ExceptionInInitializerError("[ERROR] COULDN'T INITIALIZE DRIVE SUBSYSTEM");
    }

    try {
      driverController = new PS4Controller(Constants.RobotMap.DRIVER_CONTROLLER_PORT);
      operatorController = new PS4Controller(Constants.RobotMap.OPERATOR_CONTROLLER_PORT);
    } catch (Exception err) {
      throw new ExceptionInInitializerError("[ERROR] COULDN'T INITIALIZE JOYSTICKS");
    }

    try {
      m_climberSubsystem = new ClimberSubsystem();
    } catch (Exception err) {
      throw new ExceptionInInitializerError("[ERROR] COULDN'T INITIALIZE CLIMBER COMMAND");
    }

    try {
      m_climberCommand = new ClimberCommand(m_climberSubsystem);
    } catch (Exception err) {
      throw new ExceptionInInitializerError("[ERROR] COULDN'T INITIALIZE CLIMBER SUBSYSTEM");
    }

    try {
      m_autoClimberCommand = new AutoClimberCommand(m_climberSubsystem);
    } catch (Exception err) {
      throw new ExceptionInInitializerError("[ERROR] COULDN'T INITIALIZE AUTOCLIMBER COMMAND");
    }

    m_driveSubsystem.setDefaultCommand(new SimDrive());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    autoElevatorButton.toggleWhenPressed(new AutoClimberCommand(m_climberSubsystem));

    for (int i = 0; i < 360; i += 45) {
      new POVButton(driverController, i).whileHeld(new QuickTurn(i));
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
