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
 */

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ChainedCommands.IndexerAndShoot;
import frc.robot.commands.ChainedCommands.IntakeAndIndex;
import frc.robot.commands.ChainedCommands.IntakeAndTransition;
import frc.robot.commands.ChainedCommands.ReverseAll;
import frc.robot.commands.ClimbCommands.ClimberToggle;
import frc.robot.commands.DriveCommands.OneBall;
import frc.robot.commands.DriveCommands.SimDrive;
import frc.robot.commands.IntakeCommands.IntakeToggle;
import frc.robot.commands.VisionCommands.AutoAim;
import frc.robot.subsystems.*;
import frc.robot.utility.ShuffleboardDashboard;

public class RobotContainer {

    /* GAMEPADS */
    public static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.DRIVER_CONTROLLER_PORT);
    public static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.OPERATOR_CONTROLLER_PORT);

    /* DRIVE CONTROLS */
    JoystickButton autoAimButton = new JoystickButton(driverGamepad, PS4Controller.Button.kCross.value); //X driver
    JoystickButton indexerAndShootButton = new JoystickButton(driverGamepad, PS4Controller.Button.kSquare.value); //SQUARE Driver

    /* OPERATOR CONTROLS */
    JoystickButton toggleClimber = new JoystickButton(operatorGamepad, PS4Controller.Button.kR1.value);
    JoystickButton intakeAndIndexerButton = new JoystickButton(operatorGamepad, PS4Controller.Button.kCross.value); //X Operator
    JoystickButton reverseIntake = new JoystickButton(operatorGamepad, PS4Controller.Button.kSquare.value);
    JoystickButton toggleIntakeButton = new JoystickButton(operatorGamepad, PS4Controller.Button.kCircle.value);
    POVButton shootButton = new POVButton(operatorGamepad, 0);
    JoystickButton intakeAndTransitionButton = new JoystickButton(operatorGamepad, PS4Controller.Button.kTriangle.value);

    /* SUBSYSTEMS */
    public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public static final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    public static final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final ColorSubsystem m_colorSubsystem = new ColorSubsystem();

    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(new SimDrive());
        configureButtonBindings();
        ShuffleboardDashboard.log();
    }

    private void configureButtonBindings() {
        /* DRIVER CONTROLS */
        autoAimButton.whileHeld(new AutoAim());
        indexerAndShootButton.whileHeld(new IndexerAndShoot());

        /* OPERATOR CONTROLS */
        intakeAndIndexerButton.whileHeld(new IntakeAndIndex());
        reverseIntake.whileHeld(new ReverseAll());
        toggleClimber.toggleWhenPressed(new ClimberToggle(operatorGamepad));
        toggleIntakeButton.toggleWhenPressed(new IntakeToggle());
        shootButton.whileHeld(new IndexerAndShoot());
        intakeAndTransitionButton.whileHeld(new IntakeAndTransition());
    }

    public Command getAutonomousCommand() {
        System.out.println(m_driveSubsystem.getSelectedFromChooser());
        switch (m_driveSubsystem.getSelectedFromChooser()) {
            case 0:
                return new OneBall();
            case 2:
                return m_driveSubsystem.TwoBallAuto(m_driveSubsystem);
            case 3:
                return m_driveSubsystem.ThreeBallAuto(m_driveSubsystem);
            case 4:
                return m_driveSubsystem.FourBallAuto(m_driveSubsystem);
            default:
                return null;
        }
    }
}
