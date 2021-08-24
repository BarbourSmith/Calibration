#include "calibration.h";
#include <Arduino.h>;

void printIndividual(double individual[]){
    Serial.print("| ");
    for(int i = 0; i< 9; i++){
        Serial.print(individual[i]);
        Serial.print(" | ");
    }
    Serial.print("\n");
}

void printPopulation(double population[][9]){
    
    Serial.println("Population: ");
    for(int i = 0; i < 50; i++){
        Serial.print(i);
        Serial.print("  ");
        printIndividual(population[i]);
    }
}

//Assigns an individual to the population
void assign(int index, double population[][9], double individual[]){
    for(int i = 0; i< 9; i++){
        population[index][i] = individual[i];
    }
}

//Mutates an individual within the population
void mutate(int index, double population[][9],double maxMutation){
    for(int i = 0; i< 9; i++){
        if(i != 4 && i != 5){ //Skip the (0,0) point that doesn't move
            population[index][i] = population[index][i] + (random(-100.0*maxMutation,100*maxMutation)/100.0);
        }
    }
}

double myAbs(double x){
    if(x < 0){
        return -1.0*x;
    }
    else{
        return x;
    }
}

double findDistanceToArc(double px,double py,double r,double ax,double ay){
    double i1 = px - ax;
    double i2 = py - ay;
    
    double dist = myAbs(sqrt((i1*i1)+(i2*i2))-r);
    return dist;
}

//Find the avg dist from xy point to all four arcs
double distPointToArcs(double x,double y,double measurement[],double individual[]){
    
    //Compute dist to top left arc
    double tlDist = findDistanceToArc(x, y, measurement[0], individual[0], individual[1]);
    
    //Compute dist to tr arc
    double trDist = findDistanceToArc(x, y, measurement[1], individual[2], individual[3]);
    
    //Compute dist to bl arc
    double blDist = findDistanceToArc(x, y, measurement[2], individual[4], individual[5]);
    
    //Compute dist to br arc
    double brDist = findDistanceToArc(x, y, measurement[3], individual[6], individual[7]);
    
    //Return the average
    return (tlDist + trDist + blDist + brDist)/4.0;
}

void evaluateDistandMin(double currentBest[], double x,double y,double measurement[],double individual[]){
    double newDist = distPointToArcs(x,y,measurement, individual);
    if(newDist < currentBest[2]){
        currentBest[0] = x;
        currentBest[1] = y;
        currentBest[2] = newDist;
    }
}

//Walks the gradient recursively to find the closest point
void walkClosenessGradient(double x, double y, double stepSize, double measurement[], double individual[], double finalPoint[]){
    
    double closestPoint[] = {0,0,10000};
    
    //0,0
    evaluateDistandMin(closestPoint,x,y,measurement, individual);
    //0+
    evaluateDistandMin(closestPoint,x,y+stepSize,measurement, individual);
    //++
    evaluateDistandMin(closestPoint,x+stepSize,y+stepSize,measurement, individual);
    //+0
    evaluateDistandMin(closestPoint,x+stepSize,y,measurement, individual);
    //+-
    evaluateDistandMin(closestPoint,x+stepSize,y-stepSize,measurement, individual);
    //0-
    evaluateDistandMin(closestPoint,x,y-stepSize,measurement, individual);
    //--
    evaluateDistandMin(closestPoint,x-stepSize,y-stepSize,measurement, individual);
    //-0
    evaluateDistandMin(closestPoint,x-stepSize,y,measurement, individual);
    //-+
    evaluateDistandMin(closestPoint,x-stepSize,y+stepSize,measurement, individual);
    
    //If that point is the center return
    if(closestPoint[0] == x && closestPoint[1] == y){
        finalPoint[0] = x;
        finalPoint[1] = y;
        finalPoint[2] = closestPoint[2];
    }
    else{  //Recursively continue walking the gradient
        walkClosenessGradient(closestPoint[0], closestPoint[1], stepSize, measurement, individual, finalPoint);
    }
}

//Find the dist from the closest point to the arcs
double findPointClosestToArcsDist(double measurement[], double individual[]){
    
    double currentClosestPoint[3];
    
    //Use the middle as the initial guess
    walkClosenessGradient(individual[2]/2,individual[3]/2, 100, measurement, individual, currentClosestPoint);
    
    //Second pass
    walkClosenessGradient(currentClosestPoint[0],currentClosestPoint[1], 10, measurement, individual, currentClosestPoint);
    
    //Third pass
    walkClosenessGradient(currentClosestPoint[0],currentClosestPoint[1], 1, measurement, individual, currentClosestPoint);
    
    //Fourth pass
    walkClosenessGradient(currentClosestPoint[0],currentClosestPoint[1], .1, measurement, individual, currentClosestPoint);
    
    return currentClosestPoint[2];
}

//Evaluate the fitness of an individual
void evaluateFitness(double individual[], double measurements[][4]){
    
    double m1Fitness = findPointClosestToArcsDist(measurements[0], individual);
    
    double m2Fitness = findPointClosestToArcsDist(measurements[1], individual);
    
    double m3Fitness = findPointClosestToArcsDist(measurements[2], individual);
    
    double m4Fitness = findPointClosestToArcsDist(measurements[3], individual);
    
    individual[8] = (m1Fitness + m2Fitness + m3Fitness + m4Fitness)/4.0;
}

int sort(const void *pa, const void *pb){
    
    // Need to recast the void *
    double* ia = (double*)pa;
    double* ib = (double*)pb;
    
    double a = myAbs(ia[8]);
    double b = myAbs(ib[8]);
    
    // The comparison
    return a > b ? 1 : (a < b ? -1 : 0);
    
}

void sortPopulation(double population[][9]){
    qsort(population, 50, sizeof(population[0]), sort);
}

void cullAndBreed(double population[][9],double stepSize){
    //Population is sorted at this point
    
    for(int i = 9; i < 50; i++){
        assign(i, population, population[random(0,9)]);
        mutate(i, population, stepSize);
    }
}

void evolve(double population[][9],double stepSize,double targetFitness, int timeout){
    int i = 0;
    while(population[0][8] > targetFitness){
        
        //Sort the population
        sortPopulation(population);
        
        //Breed the best (Start with just mutating them)
        cullAndBreed(population, stepSize);
        
        //Repeat until the fitness function is within some threshold or timeout
        
        i++;
        if(i > timeout){
            Serial.println("Breaking");
            break;
        }
    }
}

//Compute the calibration
void computeCalibration(double measurements[][4]){
    Serial.println("Begining to compute calibration");
    double initialWidth = 3000;
    double initialHeight = 1800;
    
    // Establish initial guesses for the corners
    double initialIndividual[] = {0, initialHeight, initialWidth, initialHeight, 0, 0, initialWidth, 0, 10000};
    
    // Build a population
    double population[50][9];
    for(int i = 0; i < 50; i++){
        assign(i, population, initialIndividual);
    }
    
    
    //Mutate the population
    for(int i = 1; i < 50; i++){
        mutate(i, population, 2);
    }
    
    //Compute fitness of individuals
    for(int i = 0; i < 50; i++){
        evaluateFitness(population[i], measurements);
    }
    
    //Evolve the population 
    evolve(population, .5, .25, 2000);
    evolve(population, .1, .01, 2000);
    
    sortPopulation(population);
    
    Serial.println("Final fittest: ");
    Serial.println(population[0][8]);

    if(population[0][8] < .2){
        Serial.println("Calibration success");
    }
    else{
        Serial.println("Calibration failure...bad measurements");
    }
    
    delay(1000);
}
