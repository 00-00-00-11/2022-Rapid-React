package frc.robot.utility;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class PS4Utility {

    /**
     *  Rumbles the PS4 Controller
     * 
     * @param controller the controller to rumble
     * @param side 0 (light), 1 (heavy)
     * @param intensity boolean from 0 to 1
     */
    public static void rumble(PS4Controller controller, ControllerRumbleType type, double intensity) {
        if (type == ControllerRumbleType.kHeavy) {
            controller.setRumble(RumbleType.kLeftRumble, intensity);
        } else if (type == ControllerRumbleType.kLight) {
            controller.setRumble(RumbleType.kRightRumble, intensity);
        }

    }
}
