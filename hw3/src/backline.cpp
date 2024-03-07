#include <vector>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <iostream>

using SolVector = Eigen::Vector<double, 2>;
using CostFunction = std::function<double(SolVector)>;
using GradientFunction = std::function<SolVector(SolVector)>;

double backtracking_line_search(CostFunction costFunction, GradientFunction grad, SolVector& start, SolVector& direction, double initialStepSize, double stepSizeReductionFactor, double minImprovementCoeff, int maxIterations = 100){
    double costAtSolution = costFunction(start);
    SolVector gradAtStart = - grad(start);
    // I interpret this as a measure of the kind of improvement we can expect from this direction
    // It is the derivative of the cost function at the start point in the direction of the search
    double improvementForSmallestStep = gradAtStart.dot(direction);

    // this version seems to assume more about the function and might not work for all cases
    // For the quadratic test, it required fewer iterations (33 iterations), so that's something.
    double stepSize = initialStepSize;
    for(int i = 0; i < maxIterations; i++){
        if(costFunction(start - stepSize * direction) < costAtSolution){
            return stepSize;
        }
        stepSize *= stepSizeReductionFactor;
    }

    // This required 34 iterations to converge
    // double stepSize = initialStepSize;
    // for(int i = 0; i < maxIterations; i++){
    //     double costAtNewSolution = costFunction(start - stepSize * direction);
    //     double expectedImprovement = minImprovementCoeff * stepSize * improvementForSmallestStep;
    //     double actualImprovement = costAtSolution - costAtNewSolution;
    //     if(actualImprovement >= expectedImprovement){
    //         return stepSize;
    //     }
    //     stepSize *= stepSizeReductionFactor;
    // }
    std::cout << "Warning: backtracking line search did not converge" << std::endl;
    return 0.0;
}

SolVector gradientDescent(CostFunction costFunction, GradientFunction grad, SolVector start, double epsilon = 1e-4){
    const double startingStepSize = 1.0;
    const double minImprovementCoeff = 0.001;
    const double stepSizeReductionFactor = 0.5;
    const int maxIterations = 100;
    int iterations = 0;
    SolVector direction = grad(start);
    while(direction.norm() > epsilon){
        double stepSize = backtracking_line_search(costFunction, grad, start, direction, startingStepSize, stepSizeReductionFactor, minImprovementCoeff, maxIterations);
        start = start - stepSize * direction;
        direction = grad(start);
        iterations++;
    }
    std::cout << "Gradient descent took " << iterations << " iterations" << std::endl;
    return start;
}

int main(){

    // Problem #2 with the quadratic boy
    // f(x) = 2x^{2}-xy-xy+y^{2} = 2x^2-2xy+y^2
    CostFunction f = [](SolVector x){
        return 2*x(0)*x(0) - 2* x(0)*x(1) + x(1)*x(1);
    };

    // Gradient of f(x)
    GradientFunction grad_f = [](SolVector x){
        SolVector grad;
        grad(0) = 4*x(0) - 2*x(1);
        grad(1) = -2*x(0) + 2*x(1);
        return grad;
    };

    // Initial guess
    SolVector x;
    x(0) = 5;
    x(1) = -10;

    // Run the gradient descent
    SolVector x_star = gradientDescent(f, grad_f, x);

    std::cout << "x_star: " << x_star << std::endl;

    return 0;
}