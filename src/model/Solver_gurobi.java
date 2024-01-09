package model;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Random;

import core.ArrayDistanceMatrix;
import core.InsertionHeuristic;
import core.NNHeuristic;
import core.OrderFirstSplitSecondHeuristic;
import core.Route;
import core.RouteAttribute;
import core.RoutePool;
import core.Split;
import dataStructures.DataHandler;
import distanceMatrices.DepotToCustomersDistanceMatrix;
import distanceMatrices.DepotToCustomersDrivingTimesMatrix;
import distanceMatrices.DepotToCustomersWalkingTimesMatrix;
import globalParameters.GlobalParameters;
import msh.AssemblyFunction;
import msh.GurobiSetPartitioningSolver;
import msh.MSH;
import msh.OrderFirstSplitSecondSampling;
import split.SplitPLRP;

/**
 * This class contains the main logic of the MSH. 
 * It contains the information of the instance and the method MSH.
 * 
 * @author nicolas.cabrera-malik
 *
 */

public class Solver_gurobi {

	// Main attributes:
	
		/**
		 * Instance name, for example "2"
		 */
	
		private String instance_name;
		
		/**
		 * Instance identifier, for example "Coordinates_3"
		 */
		
		private String instance_identifier;
	
	// Statistics 
	
		/**
		 * CPU time used to initialize the instance
		 */
		private double cpu_initialization;
		
		/**
		 * CPU time used in the sampling phase of MSH
		 */
		private double cpu_msh_sampling;

		/**
		 * CPU time used in the assembly phase of MSH
		 */
		private double cpu_msh_assembly;
	
		
	// Methods:
		
		//------------------------------------------MAIN LOGIC-----------------------------------
		
		/**
		 * This method runs the MSH
		 * @throws IOException
		 */
		public void MSH(String instance_identif)throws IOException {
			
			//0. Store main attributes: 
				
				this.instance_identifier = instance_identif;
				this.instance_name = instance_identif.replace(".txt", "");
				this.instance_name = this.instance_name.replace("Coordinates_", "");
				
			//1. Starts the clock for the initialization step:
			
				Double IniTime = (double) System.nanoTime();
				
				if(GlobalParameters.PRINT_IN_CONSOLE) {
					System.out.println("Starting the initialization step...");
				}
				
			//2. Reads the instance 
				
				// Walking speed, driving speed, etc..
				
					DataHandler data = new DataHandler(GlobalParameters.INSTANCE_FOLDER+instance_identifier);
				
				// Depot to customers distance matrix
				
					ArrayDistanceMatrix distances = null;
					distances = new DepotToCustomersDistanceMatrix(data);
	
				// Depot to customers distance matrix
					
					ArrayDistanceMatrix driving_times = null;
					driving_times = new DepotToCustomersDrivingTimesMatrix(data);
					
				// Depot to customers distance matrix
					
					ArrayDistanceMatrix walking_times = null;
					walking_times = new DepotToCustomersWalkingTimesMatrix(data);
						
			// 3. Initializes an array to store all the route pools. We will have one pool for each satellite/tspHeuristic
						
				ArrayList<RoutePool> pools = new ArrayList<RoutePool>();			
					
			// 4. Creates an assembler:
				
				AssemblyFunction assembler = null;
				assembler = new GurobiSetPartitioningSolver(data.getNbCustomers(),true,data);//GUROBI

			// 5. Initializes the MSH object with the assembler and the # of threads:
				
				MSH msh = new MSH(assembler,GlobalParameters.THREADS);	
				
			// 6. Initializes the split algorithm: 
				
				Split split = new SplitPLRP(distances,driving_times,walking_times,data);	
				
			// 7. Number of iterations for each TSP heuristc:
				
				int num_iterations = (int) Math.ceil(GlobalParameters.MSH_NUM_ITERATIONS/8);
				if(num_iterations < 1) {
					num_iterations = 1;
				}
				
			// 8. Set-up of the sampling functions: 
				
				// With a high level of randomization:
					
					this.addSamplingFunctionsHighSE(data,distances,pools,msh,split,num_iterations);
					
						
				// With a low level of randomization:
					
					this.addSamplingFunctionsLowSE(data,distances,pools,msh,split,num_iterations);
							
		//11. Stops the clock for the initialization time:
					
			Double FinTime = (double) System.nanoTime();
			cpu_initialization = (FinTime-IniTime)/1000000000;
			
			if(GlobalParameters.PRINT_IN_CONSOLE) {
				System.out.println("End of the initialization step...");
			}
			
		//12. Sets the pools:
			
			msh.setPools(pools);
			
		//13. Sampling phase of MSH:
			
			Double IniTime_msh = (double) System.nanoTime();
			
			if(GlobalParameters.PRINT_IN_CONSOLE) {
				System.out.println("Start of the sampling step...");
			}
			
			// Sampling phase:
			
			msh.run_sampling();
	
			
			if(GlobalParameters.PRINT_IN_CONSOLE) {
				System.out.println("End of the sampling step...");
			}
			
			Double FinTime_msh = (double) System.nanoTime();
			
			cpu_msh_sampling = (FinTime_msh-IniTime_msh)/1000000000;
			
	
		//15. Assembly phase of MSH:
			
			// Starts the clock:
			
			IniTime_msh = (double) System.nanoTime();
			
			if(GlobalParameters.PRINT_IN_CONSOLE) {
				System.out.println("Start of the assembly step...");
			}
			
			// Runs the assembly step:
			
			msh.run_assembly();
			
			if(GlobalParameters.PRINT_IN_CONSOLE) {
				System.out.println("End of the assembly step...");
			}
			
			// Stops the clock:
			
			FinTime_msh = (double) System.nanoTime();
			
			cpu_msh_assembly = (FinTime_msh-IniTime_msh)/1000000000;
			
		// 16. Print summary
			
			printSummary(msh,assembler,data);
						
		// 17. Print solution
			
			printSolution(msh,assembler,data);
			
		
		}
		
		
	/**
	 * This method prints the solution in console and in a txt file
	 * @param msh the msh object that contains information of the route pools
	 * @param assembler the assembler used to solve the set partitioning model
	 * @param data DataHandler object that contains information of the instance
	 */
	public void printSolution(MSH msh, AssemblyFunction assembler,DataHandler data) {
		
		// 1. Defines the path for the txt file:
		
			String path_arcs = globalParameters.GlobalParameters.RESULT_FOLDER+ "Arcs_" + instance_name +"_"+GlobalParameters.SEED+ ".txt";
			
		// 2. Prints the txt file:
			
			try {
							
				// Creates the print writer:
							
					PrintWriter pw_arcs = new PrintWriter(new File(path_arcs));
							
					System.out.println("-----------------------------------------------");
					System.out.println("Total cost: "+assembler.objectiveFunction);
					System.out.println("Routes:");	
					
				// Prints each of the selected first echelon routes:
					
					int counter = 0;	
					
					for(Route r : assembler.solution) {
						

						// Build the arcs using the chain we stored during the sampling:
						
						String chain = (String)r.getAttribute(RouteAttribute.CHAIN);
						chain = chain.replace(" 0 ", " "+(data.getNbCustomers()+1)+" ");
						chain = chain.replace("CD", ""+(data.getNbCustomers()+1));
						chain = chain.replace(" ", "");
						String[] parts = chain.split("[||]");
						
						for(int pos = 0; pos < parts.length; pos++) {
							
							if(parts[pos].length() > 0) {
								
								// Only driving:
								
									if(parts[pos].contains("->") && !parts[pos].contains("---")) {
									
										parts[pos] = parts[pos].replace("->", ";");
										String[] arcParts = parts[pos].split("[;]");
										for(int arc = 0; arc < arcParts.length-1; arc++) {
											if(!arcParts[arc].equals(arcParts[arc+1])) {
												pw_arcs.println(arcParts[arc]+";"+arcParts[arc+1]+";"+1+";"+counter);
											}
										}
										
									}
								
								// Only walking:
								
									if(!parts[pos].contains("->") && parts[pos].contains("---")) {
									
										parts[pos] = parts[pos].replace("---", ";");
										String[] arcParts = parts[pos].split("[;]");
										for(int arc = 0; arc < arcParts.length-1; arc++) {
											pw_arcs.println(arcParts[arc]+";"+arcParts[arc+1]+";"+2+";"+counter);
										}
										
									}
								
								// Mix between driving and walking:
								
									if(parts[pos].contains("->") && parts[pos].contains("---")) {
										
										parts[pos] = parts[pos].replace("---", ";");
										parts[pos] = parts[pos].replace("->", ":");
										
										int posInString = 0;
										String tail = "";
										String head = "";
										int mode = -1;
										int lastPos = -1;
										while(posInString < parts[pos].length()) {
											if(mode == -1) {
												if(parts[pos].charAt(posInString) == ':') {
													mode = 1;
													lastPos = posInString;
												}
												if(parts[pos].charAt(posInString) == ';') {
													mode = 2;
													lastPos = posInString;
												}
											}else {
												if(parts[pos].charAt(posInString) == ':') {
													if(!tail.equals(head)) {
														pw_arcs.println(tail+";"+head+";"+mode+";"+counter);	
													}
													posInString = lastPos;
													mode = -1;
													tail = "";
													head = "";
												}
												if(parts[pos].charAt(posInString) == ';') {
													if(!tail.equals(head)) {
														pw_arcs.println(tail+";"+head+";"+mode+";"+counter);	
													}
													posInString = lastPos;
													mode = -1;
													tail = "";
													head = "";
												}
											}
											
											if(mode == -1 && !(parts[pos].charAt(posInString) == ':') && !(parts[pos].charAt(posInString) == ';')) {
												tail += parts[pos].charAt(posInString);
											}else {
												if(!(parts[pos].charAt(posInString) == ':') && !(parts[pos].charAt(posInString) == ';')) {
													head += parts[pos].charAt(posInString);
												}
											}
											
											
											
											posInString++;
										}
										if(!tail.equals(head)) {
											pw_arcs.println(tail+";"+head+";"+mode+";"+counter);	
										}
										
										
									}
								
							}
							
						}
						
					// Print the route in console:
					
						System.out.println("\t\t Route "+counter+": "+r.getAttribute(RouteAttribute.CHAIN)+" - Cost: "+r.getAttribute(RouteAttribute.COST)+" - Duration: "+r.getAttribute(RouteAttribute.DURATION)+" - Service time: "+r.getAttribute(RouteAttribute.SERVICE_TIME));
						
					// Update the counter:
						
						counter++;
					}
					
					System.out.println("-----------------------------------------------");
					
			// Close the print writers:
					
				pw_arcs.close();
					
			}catch(Exception e) {
				e.printStackTrace();
				System.out.println("Error while printing the final solution");
			}
	}
	
	
	/**
	 * This method prints the summary in console and in a txt file
	 * @param msh
	 * @param assembler
	 */
	public void printSummary(MSH msh, AssemblyFunction assembler,DataHandler data) {
	
		// 1. Defines the path for the txt file:
		
			String path = globalParameters.GlobalParameters.RESULT_FOLDER+ "Summary_" + instance_name +"_"+GlobalParameters.SEED+ ".txt";
		
		// 2. Prints the txt file:
			
			try {
				
				// Creates the print writer:
				
					PrintWriter pw = new PrintWriter(new File(path));
					
				// Prints relevant information:
					
					pw.println("Instance;"+instance_name);
					pw.println("Instance_n;"+data.getNbCustomers());
					pw.println("Seed;"+GlobalParameters.SEED);
					pw.println("InitializationTime(s);"+cpu_initialization);
					pw.println("TotalTime(s);"+(cpu_msh_sampling+cpu_msh_assembly));
					pw.println("TotalDistance;"+assembler.objectiveFunction);
					pw.println("Iterations;"+msh.getNumberOfIterations());
					pw.println("SizeOfPool;"+msh.getPoolSize());
					pw.println("SamplingTime(s);"+cpu_msh_sampling);
					pw.println("AssemblyTime(s);"+cpu_msh_assembly);
				
					System.out.println("-----------------------------------------------");
					
					System.out.println("Instance: "+instance_name);
					System.out.println("Instance_n: "+data.getNbCustomers());
					System.out.println("Seed: "+GlobalParameters.SEED);
					System.out.println("InitializationTime(s): "+cpu_initialization);
					System.out.println("TotalTime(s): "+(cpu_msh_sampling+cpu_msh_assembly));
					System.out.println("TotalDistance: "+assembler.objectiveFunction);
					System.out.println("Iterations: "+msh.getNumberOfIterations());
					System.out.println("SizeOfPool: "+msh.getPoolSize());
					System.out.println("SamplingTime(s): "+cpu_msh_sampling);
					System.out.println("AssemblyTime(s): "+cpu_msh_assembly);
						
					
				// Closes the print writer:
				
					pw.close();
					
			}catch(Exception e) {
				System.out.println("Mistake printing the summary");
			}	
	}
	
	/**
	 * This methods adds the sampling functions with a high level of randomization selected by the user
	 * @param data
	 * @param distances_satellite_customers
	 * @param pools
	 * @param msh
	 * @param splits
	 * @param num_iterations
	 * @return
	 */
	public void addSamplingFunctionsHighSE(DataHandler data, ArrayDistanceMatrix distances,ArrayList<RoutePool> pools,MSH msh, Split split,int num_iterations) {

		// Sets the seed for the generation of random numbers:
		
			Random random_nn = new Random(GlobalParameters.SEED+90+1000);
			Random random_ni = new Random(GlobalParameters.SEED+100+1000);
			Random random_fi = new Random(GlobalParameters.SEED+110+1000);
			Random random_bi = new Random(GlobalParameters.SEED+120+1000);
			
		// Initializes the tsp heuristics:
		
				// RNN:
				
				NNHeuristic nn = new NNHeuristic(distances);
				nn.setRandomized(true);
				nn.setRandomGen(random_nn);
				nn.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_HIGH_RN);
				nn.setInitNode(0);
				
				
			// RNI:
				
				InsertionHeuristic ni = new InsertionHeuristic(distances,"NEAREST_INSERTION");
				ni.setRandomized(true);
				ni.setRandomGen(random_ni);
				ni.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_HIGH);
				ni.setInitNode(0);
				
				
			// RNI:
				
				InsertionHeuristic fi = new InsertionHeuristic(distances,"FARTHEST_INSERTION");
				fi.setRandomized(true);
				fi.setRandomGen(random_fi);
				fi.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_HIGH);
				fi.setInitNode(0);
			
				
			// BI:
				
				InsertionHeuristic bi = new InsertionHeuristic(distances,"BEST_INSERTION");
				bi.setRandomized(true);
				bi.setRandomGen(random_bi);
				bi.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_HIGH);
				bi.setInitNode(0);
				
				
		// Set up heuristics:
				
			OrderFirstSplitSecondHeuristic nn_h = new OrderFirstSplitSecondHeuristic(nn, split);	
			OrderFirstSplitSecondHeuristic ni_h = new OrderFirstSplitSecondHeuristic(ni, split);
			OrderFirstSplitSecondHeuristic fi_h = new OrderFirstSplitSecondHeuristic(fi, split);
			OrderFirstSplitSecondHeuristic bi_h = new OrderFirstSplitSecondHeuristic(bi, split);
			
			
		// Creates sampling functions:
				
				
			OrderFirstSplitSecondSampling f_nn = new OrderFirstSplitSecondSampling(nn_h,num_iterations,("rnn_high"));
			OrderFirstSplitSecondSampling f_ni = new OrderFirstSplitSecondSampling(ni_h,num_iterations,("rni_high"));
			OrderFirstSplitSecondSampling f_fi = new OrderFirstSplitSecondSampling(fi_h,num_iterations,("rfi_high"));
			OrderFirstSplitSecondSampling f_bi = new OrderFirstSplitSecondSampling(bi_h,num_iterations,("rbi_high"));
			
		// Creates the route pools:
				
			RoutePool pool_nn = new RoutePool();
			RoutePool pool_ni = new RoutePool();
			RoutePool pool_fi = new RoutePool();
			RoutePool pool_bi = new RoutePool();
				
		// Adds the pools:
			
			pools.add(pool_nn);
			pools.add(pool_ni);
			pools.add(pool_fi);
			pools.add(pool_bi);
			
		// Sets the route pools for each heuristic:
			
			f_nn.setRoutePool(pool_nn);
			f_ni.setRoutePool(pool_ni);
			f_fi.setRoutePool(pool_fi);
			f_bi.setRoutePool(pool_bi);
				
			
		// Adds the sampling function:
				
			msh.addSamplingFunction(f_nn);
			msh.addSamplingFunction(f_ni);
			msh.addSamplingFunction(f_fi);
			msh.addSamplingFunction(f_bi);
			
		

	}
	
	/**
	 * This methods adds the sampling functions with a low level of randomization selected by the user
	 * @param data
	 * @param distances_satellite_customers
	 * @param pools
	 * @param msh
	 * @param splits
	 * @param num_iterations
	 * @return
	 */
	public void addSamplingFunctionsLowSE(DataHandler data, ArrayDistanceMatrix distances,ArrayList<RoutePool> pools,MSH msh, Split split ,int num_iterations) {

		// Sets the seed for the generation of random numbers:
		
			Random random_nn = new Random(GlobalParameters.SEED+130+1000);
			Random random_ni = new Random(GlobalParameters.SEED+140+1000);
			Random random_fi = new Random(GlobalParameters.SEED+150+1000);
			Random random_bi = new Random(GlobalParameters.SEED+160+1000);
			
		// Initializes the tsp heuristics:
		
				// RNN:
				
				NNHeuristic nn = new NNHeuristic(distances);
				nn.setRandomized(true);
				nn.setRandomGen(random_nn);
				nn.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_LOW);
				nn.setInitNode(0);
				
				
			// RNI:
				
				InsertionHeuristic ni = new InsertionHeuristic(distances,"NEAREST_INSERTION");
				ni.setRandomized(true);
				ni.setRandomGen(random_ni);
				ni.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_LOW);
				ni.setInitNode(0);
				
				
			// RNI:
				
				InsertionHeuristic fi = new InsertionHeuristic(distances,"FARTHEST_INSERTION");
				fi.setRandomized(true);
				fi.setRandomGen(random_fi);
				fi.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_LOW);
				fi.setInitNode(0);
			
				
			// BI:
				
				InsertionHeuristic bi = new InsertionHeuristic(distances,"BEST_INSERTION");
				bi.setRandomized(true);
				bi.setRandomGen(random_bi);
				bi.setRandomizationFactor(GlobalParameters.MSH_RANDOM_FACTOR_LOW);
				bi.setInitNode(0);
				
				
		// Set up heuristics:
				
			OrderFirstSplitSecondHeuristic nn_h = new OrderFirstSplitSecondHeuristic(nn, split);	
			OrderFirstSplitSecondHeuristic ni_h = new OrderFirstSplitSecondHeuristic(ni, split);
			OrderFirstSplitSecondHeuristic fi_h = new OrderFirstSplitSecondHeuristic(fi, split);
			OrderFirstSplitSecondHeuristic bi_h = new OrderFirstSplitSecondHeuristic(bi, split);
			
			
		// Creates sampling functions:
				
				
			OrderFirstSplitSecondSampling f_nn = new OrderFirstSplitSecondSampling(nn_h,num_iterations,("rnn_high"));
			OrderFirstSplitSecondSampling f_ni = new OrderFirstSplitSecondSampling(ni_h,num_iterations,("rni_high"));
			OrderFirstSplitSecondSampling f_fi = new OrderFirstSplitSecondSampling(fi_h,num_iterations,("rfi_high"));
			OrderFirstSplitSecondSampling f_bi = new OrderFirstSplitSecondSampling(bi_h,num_iterations,("rbi_high"));
			
		// Creates the route pools:
				
			RoutePool pool_nn = new RoutePool();
			RoutePool pool_ni = new RoutePool();
			RoutePool pool_fi = new RoutePool();
			RoutePool pool_bi = new RoutePool();
				
		// Adds the pools:
			
			pools.add(pool_nn);
			pools.add(pool_ni);
			pools.add(pool_fi);
			pools.add(pool_bi);
			
		// Sets the route pools for each heuristic:
			
			f_nn.setRoutePool(pool_nn);
			f_ni.setRoutePool(pool_ni);
			f_fi.setRoutePool(pool_fi);
			f_bi.setRoutePool(pool_bi);
				
			
		// Adds the sampling function:
				
			msh.addSamplingFunction(f_nn);
			msh.addSamplingFunction(f_ni);
			msh.addSamplingFunction(f_fi);
			msh.addSamplingFunction(f_bi);
			

	}
	
	
}
