// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extra_libraries;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class GeometryUtil {
    
    /** 
     * @param startVal
     * @param endVal
     * @param t
     * @return double
     */
    protected static double doubleLerp(double startVal, double endVal, double t){
        return startVal + (endVal - startVal) * t;
    }

    
    /** 
     * @param startVal
     * @param endVal
     * @param t
     * @return Rotation2d
     */
    protected static Rotation2d rotationLerp(Rotation2d startVal, Rotation2d endVal, double t){
        return startVal.plus(endVal.minus(startVal).times(t));
    }

    
    /** 
     * @param a
     * @param b
     * @param t
     * @return Translation2d
     */
    protected static Translation2d translationLerp(Translation2d a, Translation2d b, double t){
        return a.plus((b.minus(a)).times(t));
    }

    
    /** 
     * @param a
     * @param b
     * @param c
     * @param t
     * @return Translation2d
     */
    protected static Translation2d quadraticLerp(Translation2d a, Translation2d b, Translation2d c, double t){
        Translation2d p0 = translationLerp(a, b, t);
        Translation2d p1 = translationLerp(b, c, t);
        return translationLerp(p0, p1, t);
    }

    
    /** 
     * @param a
     * @param b
     * @param c
     * @param d
     * @param t
     * @return Translation2d
     */
    protected static Translation2d cubicLerp(Translation2d a, Translation2d b, Translation2d c, Translation2d d, double t){
        Translation2d p0 = quadraticLerp(a, b, c, t);
        Translation2d p1 = quadraticLerp(b, c, d, t);
        return translationLerp(p0, p1, t);
    }
}