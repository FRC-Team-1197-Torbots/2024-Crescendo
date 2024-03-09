package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;

public class VectorGen {

    private double xComp;
    private double yComp;

    public VectorGen(Translation2d coords){
        xComp = coords.getX();
        yComp = coords.getY();
    }

    public VectorGen(Translation2d initCoords, Translation2d finalCoords){
        xComp = finalCoords.getX() - initCoords.getX();
        yComp = finalCoords.getY() - initCoords.getY();
    }

    public VectorGen(double angle){
        xComp = Math.cos(Math.toRadians(angle));
        yComp = Math.sin(Math.toRadians(angle));
    }

    public double getMagnitude(){
        return Math.sqrt((Math.pow(xComp, 2)) + (Math.pow(yComp, 2)));
    }

    public double getXComp(){
        return xComp;
    }

    public double getYComp(){
        return yComp;
    }
    
}
