// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;  
import frc.robot.IndexerCommand;  
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel;
//  import frc.robot.commands.IndexerSubsystem;
 



 




public class IndexerSubsystem extends SubsystemBase { 
  CANSparkMax indexMotor = new CANSparkMax(frc.robot.Constants.DriveConstants.INDEX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);  
  
  


  
  

  
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {  
      

    indexMotor.set(0);

    
    
    

    
  } 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
