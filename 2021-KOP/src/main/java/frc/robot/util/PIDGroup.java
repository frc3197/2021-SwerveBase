// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class PIDGroup {

    public double p, i, d, ks, kv;

    public PIDGroup(double p, double i, double d) {
        this.p = p;
        this.d = d;
        this.i = i;
    }

    public PIDGroup(double p, double i, double d, double ks, double kv) {
        this.p = p;
        this.d = d;
        this.i = i;
        this.ks = ks;
        this.kv = kv;
    }

}
