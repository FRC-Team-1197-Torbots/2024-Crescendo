package frc.robot.utils;

import edu.wpi.first.math.Vector;

public class VectorTools {

    public static double dotProduct(VectorGen u, VectorGen v){
        return ((u.getXComp()*v.getXComp()) + (u.getYComp()*v.getYComp()));
    }

    public static double getAngle(VectorGen u, VectorGen v){
        double angleCosine = (dotProduct(u, v)) / (u.getMagnitude() * v.getMagnitude());
        return Math.acos(angleCosine);
    }
    
}
