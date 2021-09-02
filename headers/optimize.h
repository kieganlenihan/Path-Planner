#ifndef OPTIMIZE
#define OPTIMIZE

// Working directory headers
#include "path_planner.h"               // type definitions and CGAL Kernels
#include "mover.h"                      // path calculations

// Sub-scopes
using namespace std;

class Gene: public Mover {
    private:
        double theta, x, y;
    public:
        Transformation set_gene();

};
class Chromosome: public Gene {
    private:
        
    public:
        double evaluate_fitness();
};

class Population: public Chromosome {
    private:
        int pop_size, n_survivors;
        double mutation_rate;
    public:
        void set_genetic_algo_parameters(int ps, int ns, double mr);
        vector<Chromosome> initialize_population();
        Chromosome best_individual;
        

};

#endif