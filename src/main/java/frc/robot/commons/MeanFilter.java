package frc.robot.commons;

import java.util.ArrayList;

public class MeanFilter {

    private final int size; 
    private ArrayList<Double> orderedValues; 

    public MeanFilter(int size) {
        this.size = size;
    }

    public double calculate(double next) {
        orderedValues.add(next);
        if (orderedValues.size() > size) {
            orderedValues.remove(0);
        }
        double total = 0.0;
        int bufferSize = orderedValues.size(); 
        for (double val : orderedValues) {
            total += val;
        }
        return total/bufferSize;
    }
    
}
