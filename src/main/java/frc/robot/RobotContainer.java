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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static final PS4Controller driverGamepad = new PS4Controller(0);
  public static final PS4Controller operatorGamepad = new PS4Controller(1);
  public static final PS4Controller climberGamepad = new PS4Controller(2);

  JoystickButton indxerAndShootButton = new JoystickButton(operatorGamepad, 1);
  JoystickButton toggleIntakeButton = new JoystickButton(operatorGamepad, 3);
  JoystickButton intakeAndIndexerButton = new JoystickButton(operatorGamepad, 2);

  JoystickButton manualClimbToggle = new JoystickButton(climberGamepad, PS4Controller.Button.kSquare.value);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    manualClimbToggle.toggleWhenPressed(new ClimberManual());

    toggleIntakeButton.toggleWhenPressed(new IntakeToggle());
    intakeAndIndexerButton.whileHeld(new IntakeAndIndex());
    indxerAndShootButton.whileHeld(new IndexerAndShoot());
    
    for (int i = 0; i < 360; i += 45) {
      new POVButton(driverGamepad, i).whileHeld(new QuickTurn(i));
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
