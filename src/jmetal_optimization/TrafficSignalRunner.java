package jmetal_optimization;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import org.uma.jmetal.algorithm.singleobjective.geneticalgorithm.GeneticAlgorithmBuilder;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.crossover.HUXCrossover;
import org.uma.jmetal.operator.impl.crossover.SBXCrossover;
import org.uma.jmetal.operator.impl.mutation.BitFlipMutation;
import org.uma.jmetal.operator.impl.mutation.PolynomialMutation;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.BinaryProblem;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.BinarySolution;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.AlgorithmRunner;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.comparator.RankingAndCrowdingDistanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.MultithreadedSolutionListEvaluator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class to configure and run a generational genetic algorithm. The target problem TrafficSignalGA.
 *
 * @author Tarik Reza Toha <1017052013@grad.cse.buet.ac.bd>
 */
public class TrafficSignalRunner extends AbstractAlgorithmRunner {


    private DoubleProblem problem;
    private CrossoverOperator<DoubleSolution> crossover;
    private MutationOperator<DoubleSolution> mutation;
    private SelectionOperator<List<DoubleSolution>, DoubleSolution> selection;
    private AlgorithmRunner algorithmRunner;
    private long computingTime;

    public static void main(String[] args) {
        // new TrafficSignalRunner().optimizerGA();
        List<Integer> trafficJam_Motorized = Arrays.asList(10, 20, 15, 500);
        List<Integer> trafficJam_NonMotorized = Arrays.asList(5, 3, 4, 60);
       new TrafficSignalRunner().optimizerNSGAII(4,0, 100, trafficJam_Motorized, trafficJam_NonMotorized);
    //    new TrafficSignalRunner().optimizerNSGAIIBinary();
    }


    public List<Integer> optimizerNSGAII(int numberOfLinks, int minJamTime, int maxJamTime, List<Integer> trafficJam_Motorized, List<Integer> trafficJam_NonMotorized) {
        DoubleProblem problem = new TrafficSignalNSGAII(numberOfLinks, minJamTime, maxJamTime, trafficJam_Motorized, trafficJam_NonMotorized);

        CrossoverOperator<DoubleSolution> crossover = new SBXCrossover(1.0 / problem.getNumberOfVariables(), 20.0);

        MutationOperator<DoubleSolution> mutation = new PolynomialMutation(1.0 / problem.getNumberOfVariables(), 100);

        SelectionOperator<List<DoubleSolution>, DoubleSolution> selection = new BinaryTournamentSelection<>(new RankingAndCrowdingDistanceComparator<>());
        SolutionListEvaluator<DoubleSolution> evaluator = new MultithreadedSolutionListEvaluator<>(4, problem);

        Algorithm<List<DoubleSolution>> algorithm = new NSGAIIBuilder<>(problem, crossover, mutation)
                .setSelectionOperator(selection)
                .setSolutionListEvaluator(evaluator)
                .setPopulationSize(50)
                .setMaxEvaluations(10000)
                .build();

        algorithmRunner = new AlgorithmRunner.Executor(algorithm).execute();

        computingTime = algorithmRunner.getComputingTime();
        // JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");

        List<DoubleSolution> population = algorithm.getResult();
        List<Integer> signalCycle = selectSolution_WeightedSum(population);
        // System.out.println("Optimized signal cycle: " + signalCycle);        
        // printFinalSolutionSet(population);
        return signalCycle;
    }

    private List<Integer> selectSolution_WeightedSum(List<? extends Solution<?>> population){
        System.out.println("Hi");
        List<Double> weightedSum = new ArrayList<>();
        double[] weights = {0.5, 0.5};
        for (int i = 0; i < population.size(); i++) {
            double f1 = population.get(i).getObjective(0);
            double f2 = population.get(i).getObjective(0);
            double F = weights[0]*f1 + weights[1]*f2;
            weightedSum.add(F);
        }

        // Find the minimum value and its index
        double minValue = Double.MAX_VALUE;
        int minIndex = -1;
        for (int i = 0; i < weightedSum.size(); i++) {
            if (weightedSum.get(i) < minValue) {
                minValue = weightedSum.get(i);
                minIndex = i;
            }
        }

        //get signal cycle at index minIndex
        List<Integer> signalCycle = new ArrayList<>();
        int numberOfLinks = population.get(0).getNumberOfVariables();
        for (int i=0; i<numberOfLinks; i++){
            Double double_signalValue =(Double) population.get(minIndex).getVariableValue(i);
            int int_signalValue = double_signalValue.intValue();
            signalCycle.add(int_signalValue);
        }
        // System.out.println(signalCycle);
        return signalCycle;
    }

}
