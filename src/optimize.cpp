// System headers
#include <math.h>                       // sin, cos

// Working directory headers
#include "headers/optimize.h"           // type definitions and CGAL Kernels

Transformation Gene::set_gene() {
    Transformation transform(cos(theta), -sin(theta), x, sin(theta), cos(theta), y, 1);
    return transform;
}

void Population::set_genetic_algo_parameters(int ps, int ns, double mr) {
    pop_size = ps;
    n_survivors = ns;
    mutation_rate = mr;
}

