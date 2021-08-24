
void printIndividual(double individual[]);
void printPopulation(double population[][9]);
void assign(int index, double population[][9], double individual[]);
void mutate(int index, double population[][9],double maxMutation);
double findPointClosestToArcsDist(double measurement[], double individual[]);
void evaluateFitness(double individual[], double measurements[][4]);
void computeCalibration(double measurements[][4]);
