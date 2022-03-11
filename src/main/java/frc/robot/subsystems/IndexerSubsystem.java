/***
 *
 *                                          dddddddd                                                                                                                    bbbbbbbb
 *    IIIIIIIIII                            d::::::d                                                                                   SSSSSSSSSSSSSSS                  b::::::b                                                                           tttt
 *    I::::::::I                            d::::::d                                                                                 SS:::::::::::::::S                 b::::::b                                                                        ttt:::t
 *    I::::::::I                            d::::::d                                                                                S:::::SSSSSS::::::S                 b::::::b                                                                        t:::::t
 *    II::::::II                            d:::::d                                                                                 S:::::S     SSSSSSS                  b:::::b                                                                        t:::::t
 *      I::::Innnn  nnnnnnnn        ddddddddd:::::d     eeeeeeeeeeee  xxxxxxx      xxxxxxx eeeeeeeeeeee    rrrrr   rrrrrrrrr        S:::::S            uuuuuu    uuuuuu  b:::::bbbbbbbbb        ssssssssssyyyyyyy           yyyyyyy  ssssssssss   ttttttt:::::ttttttt        eeeeeeeeeeee       mmmmmmm    mmmmmmm
 *      I::::In:::nn::::::::nn    dd::::::::::::::d   ee::::::::::::ee x:::::x    x:::::xee::::::::::::ee  r::::rrr:::::::::r       S:::::S            u::::u    u::::u  b::::::::::::::bb    ss::::::::::sy:::::y         y:::::y ss::::::::::s  t:::::::::::::::::t      ee::::::::::::ee   mm:::::::m  m:::::::mm
 *      I::::In::::::::::::::nn  d::::::::::::::::d  e::::::eeeee:::::eex:::::x  x:::::xe::::::eeeee:::::eer:::::::::::::::::r       S::::SSSS         u::::u    u::::u  b::::::::::::::::b ss:::::::::::::sy:::::y       y:::::yss:::::::::::::s t:::::::::::::::::t     e::::::eeeee:::::eem::::::::::mm::::::::::m
 *      I::::Inn:::::::::::::::nd:::::::ddddd:::::d e::::::e     e:::::e x:::::xx:::::xe::::::e     e:::::err::::::rrrrr::::::r       SS::::::SSSSS    u::::u    u::::u  b:::::bbbbb:::::::bs::::::ssss:::::sy:::::y     y:::::y s::::::ssss:::::stttttt:::::::tttttt    e::::::e     e:::::em::::::::::::::::::::::m
 *      I::::I  n:::::nnnn:::::nd::::::d    d:::::d e:::::::eeeee::::::e  x::::::::::x e:::::::eeeee::::::e r:::::r     r:::::r         SSS::::::::SS  u::::u    u::::u  b:::::b    b::::::b s:::::s  ssssss  y:::::y   y:::::y   s:::::s  ssssss       t:::::t          e:::::::eeeee::::::em:::::mmm::::::mmm:::::m
 *      I::::I  n::::n    n::::nd:::::d     d:::::d e:::::::::::::::::e    x::::::::x  e:::::::::::::::::e  r:::::r     rrrrrrr            SSSSSS::::S u::::u    u::::u  b:::::b     b:::::b   s::::::s        y:::::y y:::::y      s::::::s            t:::::t          e:::::::::::::::::e m::::m   m::::m   m::::m
 *      I::::I  n::::n    n::::nd:::::d     d:::::d e::::::eeeeeeeeeee     x::::::::x  e::::::eeeeeeeeeee   r:::::r                             S:::::Su::::u    u::::u  b:::::b     b:::::b      s::::::s      y:::::y:::::y          s::::::s         t:::::t          e::::::eeeeeeeeeee  m::::m   m::::m   m::::m
 *      I::::I  n::::n    n::::nd:::::d     d:::::d e:::::::e             x::::::::::x e:::::::e            r:::::r                             S:::::Su:::::uuuu:::::u  b:::::b     b:::::bssssss   s:::::s     y:::::::::y     ssssss   s:::::s       t:::::t    tttttte:::::::e           m::::m   m::::m   m::::m
 *    II::::::IIn::::n    n::::nd::::::ddddd::::::dde::::::::e           x:::::xx:::::xe::::::::e           r:::::r                 SSSSSSS     S:::::Su:::::::::::::::uub:::::bbbbbb::::::bs:::::ssss::::::s     y:::::::y      s:::::ssss::::::s      t::::::tttt:::::te::::::::e          m::::m   m::::m   m::::m
 *    I::::::::In::::n    n::::n d:::::::::::::::::d e::::::::eeeeeeee  x:::::x  x:::::xe::::::::eeeeeeee   r:::::r                 S::::::SSSSSS:::::S u:::::::::::::::ub::::::::::::::::b s::::::::::::::s       y:::::y       s::::::::::::::s       tt::::::::::::::t e::::::::eeeeeeee  m::::m   m::::m   m::::m
 *    I::::::::In::::n    n::::n  d:::::::::ddd::::d  ee:::::::::::::e x:::::x    x:::::xee:::::::::::::e   r:::::r                 S:::::::::::::::SS   uu::::::::uu:::ub:::::::::::::::b   s:::::::::::ss       y:::::y         s:::::::::::ss          tt:::::::::::tt  ee:::::::::::::e  m::::m   m::::m   m::::m
 *    IIIIIIIIIInnnnnn    nnnnnn   ddddddddd   ddddd    eeeeeeeeeeeeeexxxxxxx      xxxxxxx eeeeeeeeeeeeee   rrrrrrr                  SSSSSSSSSSSSSSS       uuuuuuuu  uuuubbbbbbbbbbbbbbbb     sssssssssss        y:::::y           sssssssssss              ttttttttttt      eeeeeeeeeeeeee  mmmmmm   mmmmmm   mmmmmm
 *                                                                                                                                                                                                              y:::::y
 *                                                                                                                                                                                                             y:::::y
 *                                                                                                                                                                                                            y:::::y
 *                                                                                                                                                                                                           y:::::y
 *                                                                                                                                                                                                          yyyyyyy
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.SparkMaxUtility;

public class IndexerSubsystem extends SubsystemBase {

  private CANSparkMax transitionMotor;
  private CANSparkMax beltMotor;

  public IndexerSubsystem() {
    transitionMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.INDEXER_TRANSITION_CAN, true);
    beltMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.INDEXER_BELT_CAN, true);
    
  }

  public void runIndexer(double speed) {
    SparkMaxUtility.runSparkMax(transitionMotor, speed);
    SparkMaxUtility.runSparkMax(beltMotor, -speed);
  } 

}
