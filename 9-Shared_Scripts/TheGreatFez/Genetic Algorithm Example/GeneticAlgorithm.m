PlanetSelect;
% Final answer we are looking for
Generation_Limit = 50;
Pop_Size = 25;
Bit_Size = 32;
% Chance that a bit will mutate and overall size of mutation
Mutation_Rate = 10/100;
Mutation_Size = 10/100;
% Population indexes
Max_Bit = 2^Bit_Size-1;
Genes_Total = 5;
% Gene minimal values and range of values
Gene_Range = zeros(Genes_Total,2);
Gene_Range = [Min_Throttle,Max_Throttle-Min_Throttle;
              100,2400;
              0,30;              
              30,350;
              0,.75];
% Setting up the children array
fitness = Genes_Total + 1;
Array_Size = fitness;
population = zeros(Pop_Size,Array_Size);
children = zeros(Pop_Size,Array_Size);
Std_Dev = zeros(Generation_Limit+1,1);
Best_Fit_Data = zeros(Generation_Limit,1);
Mom_Fit_Data = zeros(Generation_Limit,1);
% Start the initial population
fprintf('Initialization\n');
for i = 1:Pop_Size
    fitness_check = 0;
    while fitness_check == 0
        for j = 1:Genes_Total
            children(i,j) = Gene_Range(j,1) + rand*Gene_Range(j,2);
        end
        gen = i;
        DeltaVLeft = 0;
        ecc_check = 0;
        RunSim;
        fitness_check = Fitness(DeltaVLeft, ecc_check, Ra_check, TargetOrbit);
    end
    fprintf('  Candidate %i Finished\n',i);
    children(i,fitness) = fitness_check;
end
Std_Dev(1) = std(children(:,fitness));
fprintf('  Standard Deviation = %f\n',Std_Dev(1));
Generations = 0;
Mom_Fit = 0;
Dad_Fit = 0;
Best_Fit = 0;
%After initial population, run through generations to optimize parameters
for i = 1:Generation_Limit
    Mutation_Rate = max((10-10*(Generations/Generation_Limit)),1)/100;
    Mutation_Size = max((10-10*(Generations/Generation_Limit)),1)/100;
    Generations = Generations + 1;

    fprintf('Generation %i\n',i);
    population = children;
    %Select the best and second best from the population for mating
    %NOTE: This is not the best solution, next iteration of this algoritm is to select
    %two randomly within the population and create only one child at a time while eliminating
    %the worst in the population
    SortedFitness = sort(population(:,fitness));
    Mom_Fit = SortedFitness(Pop_Size);
    Dad_Fit = SortedFitness(Pop_Size - 1);
    if Mom_Fit > Best_Fit
      Best_Fit = Mom_Fit;
    end
    Mom_Fit_Data(i) = Mom_Fit;
    Best_Fit_Data(i) = Best_Fit;
    Mom = -1;
    Dad = -1;
    %My solution to finding the best scoring parent and second best, there
    %is probably a better way to do this but it works
    while ((Mom < 0) & (Dad < 0) )

      for j = 1:Pop_Size
          check = j;
          if (Mom_Fit == population(check,fitness))
              Mom = population(check,:);
              if Mom_Fit == Best_Fit
                  Best = Mom;
              end
          end

          if (Dad_Fit == population(check,fitness))

              Dad = population(check,:);
          end

      end

    end
    % End condition for the generation
    %   if Mom_Fit <= Target
    %       break
    %   end
    
    %Create new population using Best and Second best from the population
    for j = 1:Pop_Size
      fitness_check = 0;
      while fitness_check == 0  
        Child = Mate(Mom,Dad,Bit_Size,Genes_Total,Gene_Range,Array_Size,Mutation_Rate,Mutation_Size);
        children(j,:) = Child;
        gen = j;
        DeltaVLeft = 0;
        ecc_check = 0;
        RunSim;
        fitness_check = Fitness(DeltaVLeft, ecc_check, Ra_check, TargetOrbit);
      end
      fprintf('  Child %i Finished\n',j);
      children(j,fitness) = fitness_check;

    end
    Std_Dev(i+1) = std(children(:,fitness));
    fprintf('  Standard Deviation = %f\n',Std_Dev(i+1));
end
plotting_x =[1:1:Generation_Limit]';
Average = mean(population(:,1:3));