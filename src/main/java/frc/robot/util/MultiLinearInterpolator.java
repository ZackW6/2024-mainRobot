// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class MultiLinearInterpolator {

    private InterpolatingTreeMap<Double,Double>[] multiLinearInterpolator;

    /**
     * key values first, followed by corresponding values for each row, must have at least two layers
     * @param matrix
     */
    public MultiLinearInterpolator(double[][] matrix){
        if (matrix.length<2){
            throw new IllegalArgumentException("Matrix should have at least two layers for interpolation");
        }
        multiLinearInterpolator = new InterpolatingTreeMap[matrix[0].length-1];

        for (int i = 0; i<multiLinearInterpolator.length;i++){
            multiLinearInterpolator[i] = new InterpolatingTreeMap<Double,Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
            for (int y = 0; y<matrix.length;y++){
                multiLinearInterpolator[i].put(matrix[y][0],matrix[y][i+1]);
            }
        }
    }
    public double[] get(double key){
        double[] values = new double[multiLinearInterpolator.length];
        for(int i = 0; i<multiLinearInterpolator.length;i++){
            values[i] = multiLinearInterpolator[i].get(key);
        }
        return values;
    }
}
