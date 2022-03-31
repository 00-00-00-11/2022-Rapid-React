package frc.robot.simulation;

import frc.robot.vision.Limelight;

public class LimelightSim extends Limelight {

    double tx = 0.0;
    double ty = 0.0;

    public LimelightSim(double shooterAngle, double limelightHeight, double goalHeight, double pipeline, double tx,
            double ty) {
        super(shooterAngle, limelightHeight, goalHeight, pipeline);
        this.tx = tx;
        this.ty = ty;
    }

    public void setTx(double tx) {
        this.tx = tx;
    }

    public void setTy(double ty) {
        this.ty = ty;
    }

    @Override
    public double getTx() {
        if(tx > 0.0) {
            tx -= 0.1;
        } else {
            tx += 0.1;
        }
        
        return tx;
    }

    @Override
    public double getTy() {
        return ty;
    }

}
