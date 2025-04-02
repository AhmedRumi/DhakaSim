package jmetal_optimization;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrafficSignalNSGAII extends AbstractDoubleProblem {
    private int trafficJam_Motorized[] ;
    private int trafficJam_NonMotorized[];
    /**
     * speed in terms of exporting unit traffic volume per second.
     */
    private double heteroSpeed_Motorized = 20;
    private double heteroSpeed_NonMotorized = 3;

    /**
     * Constructor Creates a default instance of the traffic signal optimization
     * problem
     *
     * @param numberOfLinks Number of links of the junction
     * @param minJamTime    Minimum time of traffic jam
     * @param maxJamTime    Maximum time of traffic jam
     */
    public TrafficSignalNSGAII(int numberOfLinks, double minJamTime, double maxJamTime, List<Integer> jam_Motorized, List<Integer> jam_NonMotorized) {
        setNumberOfVariables(numberOfLinks);
        setNumberOfObjectives(2);
        setName("TrafficSignal");

        trafficJam_Motorized = new int[jam_Motorized.size()];
        for (int i = 0; i < jam_Motorized.size(); i++) {
            trafficJam_Motorized[i] = jam_Motorized.get(i);
        }
        // System.out.println("Check array copy: " + jam_Motorized + " " + Arrays.toString(trafficJam_Motorized));
        trafficJam_NonMotorized = new int[jam_NonMotorized.size()];
        for (int i = 0; i < jam_NonMotorized.size(); i++) {
            trafficJam_NonMotorized[i] = jam_NonMotorized.get(i);
        }
        // System.out.println("Check array copy: " + jam_NonMotorized + " " + Arrays.toString(trafficJam_NonMotorized));

        List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables());
        List<Double> upperLimit = new ArrayList<>(getNumberOfVariables());

        for (int i = 0; i < getNumberOfVariables(); i++) {
            lowerLimit.add(minJamTime);
            upperLimit.add(maxJamTime);
        }

        setLowerLimit(lowerLimit);
        setUpperLimit(upperLimit);


        double sum = 0;
        for (int i=0; i<getNumberOfVariables(); i++) {
            sum += trafficJam_Motorized[i] + trafficJam_NonMotorized[i];
        }
        System.out.println(Arrays.toString(trafficJam_Motorized) + "; " + Arrays.toString(trafficJam_NonMotorized) + "; " + sum);
    }

    @Override
    public void evaluate(DoubleSolution solution) {
        double[] objectives = new double[getNumberOfObjectives()];

        int numberOfVariables = getNumberOfVariables();
        double[] candidate = new double[numberOfVariables];
        for (int var = 0; var < numberOfVariables; var++) {
            candidate[var] = solution.getVariableValue(var);
        }
        
        int[] updatedJam_Motorized = new int[numberOfVariables];
        int[] updatedJam_NonMotorized = new int[numberOfVariables];
        System.arraycopy(trafficJam_Motorized, 0, updatedJam_Motorized, 0, numberOfVariables);
        System.arraycopy(trafficJam_NonMotorized, 0, updatedJam_NonMotorized, 0, numberOfVariables);
        double sumJam_Motorized = 0.0, sumJam_NonMotorized = 0.0;
        double sumTime = 0.0;
        for (int var = 0; var < numberOfVariables; var++) {
            // #consider effect of heterogeneous (motorized and nonmotorized) vehicle
            double effective_heteroSpeed_Motorized = .25*heteroSpeed_Motorized*updatedJam_Motorized[var]/(updatedJam_Motorized[var] + updatedJam_NonMotorized[var]);
            double motorized_passed_intersection = candidate[var] * effective_heteroSpeed_Motorized;
            updatedJam_Motorized[var] = updatedJam_Motorized[var] - (int) motorized_passed_intersection;
            updatedJam_Motorized[var] = updatedJam_Motorized[var] < 0? 0 : updatedJam_Motorized[var];
            sumJam_Motorized += updatedJam_Motorized[var];

            // System.out.println("Debug: " + Arrays.toString(updatedJam_Motorized) + ", " + Arrays.toString(updatedJam_NonMotorized));
            // System.out.println("Debug: " + Arrays.toString(heteroSpeed_Motorized) + ", " + effective_heteroSpeed_Motorized);

            double effective_heteroSpeed_NonMotorized = .25*heteroSpeed_NonMotorized*updatedJam_NonMotorized[var]/(updatedJam_Motorized[var] + updatedJam_NonMotorized[var]);
            double nonMotorized_passed_intersection = candidate[var] * effective_heteroSpeed_NonMotorized;
            updatedJam_NonMotorized[var] = updatedJam_NonMotorized[var] - (int) nonMotorized_passed_intersection;
            updatedJam_NonMotorized[var] = updatedJam_NonMotorized[var] < 0? 0 : updatedJam_NonMotorized[var];
            sumJam_NonMotorized += updatedJam_NonMotorized[var];

            for (int i = 0; i < numberOfVariables; i++) {
                if (i != var) {
                    // double time_For_Motorized = seriesSum(updatedJam_Motorized[i]) / (1 + updatedJam_Motorized[i]) / heteroSpeed_Motorized[i];
                    // double time_For_NonMotorized = seriesSum(updatedJam_NonMotorized[i]) / (1 + updatedJam_NonMotorized[i]) / heteroSpeed_NonMotorized[i];
                    sumTime += candidate[var];
                    // + Math.max(time_For_Motorized, time_For_NonMotorized);
                }
            }
        }

        objectives[0] = sumJam_Motorized + sumJam_NonMotorized ;
        objectives[1] = sumTime;
        solution.setObjective(0, objectives[0]);
        solution.setObjective(1, objectives[1]);
    }

    private double seriesSum(double max) {
        double sum = 0;
        max = Math.round(max) - 1;
        for (int i = 1; i <= max; i++) {
            sum += i;
        }
        return sum;
    }
}
