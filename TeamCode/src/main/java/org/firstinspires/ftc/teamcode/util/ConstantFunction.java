package org.firstinspires.ftc.teamcode.util;

import java.util.function.Function;

public class ConstantFunction {

    public static final int TIME_INTERVAL = 1000; // Standard kinematic time interval in ms
    private static final float sqrt2 = 1.41421356237f;

    // Probably could just be done with functions but this is more Fun(ction)

    // Velocity functions, interval = 100ms
    public static final Function<Double , Double> POLY2_COLLAPSE = x -> x < TIME_INTERVAL ? Math.pow(x / TIME_INTERVAL , 2) / 2 : 1;
    public static final Function<Double , Double> LIN_COLLAPSE = x -> x < TIME_INTERVAL ? x / TIME_INTERVAL : 1;
}
