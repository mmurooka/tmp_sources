#include <fstream>
#include <iostream>

#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_ipm/IpmSolver.h>

using namespace ocs2;

constexpr size_t STATE_DIM = 2;
constexpr size_t INPUT_DIM = 1;

/** \brief Dynamics of Van der Pol oscillator.

    See https://web.casadi.org/docs/#a-simple-test-problem
 */
class OscillatorDynamics : public SystemDynamicsBase {
public:
  OscillatorDynamics() {}

  OscillatorDynamics *clone() const override {
    return new OscillatorDynamics(*this);
  }

  virtual vector_t computeFlowMap(scalar_t t, const vector_t &x,
                                  const vector_t &u,
                                  const PreComputation &preComp) override {
    vector_t x_dot(STATE_DIM);
    x_dot << (1.0 - std::pow(x[1], 2)) * x[0] - x[1] + u[0], x[0];
    return x_dot;
  }

  virtual VectorFunctionLinearApproximation
  linearApproximation(scalar_t t, const vector_t &x, const vector_t &u,
                      const PreComputation &preComp) override {
    VectorFunctionLinearApproximation func =
        VectorFunctionLinearApproximation::Zero(STATE_DIM, STATE_DIM,
                                                INPUT_DIM);
    func.f = computeFlowMap(t, x, u, preComp);
    func.dfdx(0, 0) = 1.0 - std::pow(x[1], 2);
    func.dfdx(0, 1) = -2 * x[0] * x[1] - 1.0;
    func.dfdx(1, 0) = 1;
    func.dfdu(0, 0) = 1;
    return func;
  }
};

/** \brief State cost. */
class OscillatorCost final : public QuadraticStateInputCost {
public:
  OscillatorCost()
      : QuadraticStateInputCost(matrix_t::Identity(STATE_DIM, STATE_DIM),
                                matrix_t::Identity(INPUT_DIM, INPUT_DIM)) {}
  OscillatorCost *clone() const override { return new OscillatorCost(*this); }
};

/** \brief Terminal cost. */
class OscillatorFinalCost final : public QuadraticStateCost {
public:
  OscillatorFinalCost()
      : QuadraticStateCost(matrix_t::Identity(STATE_DIM, STATE_DIM)) {}

  OscillatorFinalCost *clone() const override {
    return new OscillatorFinalCost(*this);
  }
};

int main() {
  // Instantiate problem
  OptimalControlProblem problem;
  {
    problem.dynamicsPtr.reset(new OscillatorDynamics());
    problem.costPtr->add("cost", std::make_unique<OscillatorCost>());
    // problem.finalCostPtr->add("finalCost",
    // std::make_unique<OscillatorFinalCost>());

    auto getStateBoxConstraint = [&](const vector_t &minState,
                                     const vector_t &maxState) {
      constexpr size_t numIneqConstraint = 2 * STATE_DIM;
      const vector_t e =
          (vector_t(numIneqConstraint) << -minState, maxState).finished();
      const matrix_t C = (matrix_t(numIneqConstraint, STATE_DIM)
                              << matrix_t::Identity(STATE_DIM, STATE_DIM),
                          -matrix_t::Identity(STATE_DIM, STATE_DIM))
                             .finished();
      return std::make_unique<LinearStateConstraint>(std::move(e),
                                                     std::move(C));
    };
    const vector_t xmin = (vector_t(STATE_DIM) << -1e3, -0.05).finished();
    const vector_t xmax = (vector_t(STATE_DIM) << 1e3, 1e3).finished();
    problem.stateInequalityConstraintPtr->add(
        "xbound", getStateBoxConstraint(xmin, xmax));

    auto getInputBoxConstraint = [&](vector_t minInput, vector_t maxInput) {
      constexpr size_t numIneqConstraint = 2 * INPUT_DIM;
      const vector_t e =
          (vector_t(numIneqConstraint) << -minInput, maxInput).finished();
      const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
      const matrix_t D = (matrix_t(numIneqConstraint, INPUT_DIM)
                              << matrix_t::Identity(INPUT_DIM, INPUT_DIM),
                          -matrix_t::Identity(INPUT_DIM, INPUT_DIM))
                             .finished();
      return std::make_unique<LinearStateInputConstraint>(
          std::move(e), std::move(C), std::move(D));
    };
    const vector_t umin = (vector_t(INPUT_DIM) << -1.0).finished();
    const vector_t umax = (vector_t(INPUT_DIM) << 0.9).finished();
    problem.inequalityConstraintPtr->add("ubound",
                                         getInputBoxConstraint(umin, umax));
  }

  // Instantiate solver
  std::unique_ptr<IpmSolver> solver;
  {
    const auto settings = []() {
      ipm::Settings s;
      s.dt = 0.01;
      s.ipmIteration = 10;
      // s.useFeedbackPolicy = true;
      // s.printSolverStatistics = true;
      // s.printSolverStatus = true;
      // s.printLinesearch = true;
      // s.printSolverStatistics = true;
      // s.printSolverStatus = true;
      // s.printLinesearch = true;
      s.nThreads = 1;
      s.initialBarrierParameter = 1.0e-02;
      s.targetBarrierParameter = 1.0e-04;
      s.barrierLinearDecreaseFactor = 0.2;
      s.barrierSuperlinearDecreasePower = 1.5;
      s.fractionToBoundaryMargin = 0.995;
      return s;
    }();
    DefaultInitializer zeroInitializer(INPUT_DIM);
    solver = std::make_unique<IpmSolver>(settings, problem, zeroInitializer);
  }

  // Set reference
  {
    std::shared_ptr<ReferenceManager> referenceManagerPtr;
    const vector_t x = vector_t::Zero(2);
    const vector_t u = vector_t::Zero(1);
    TargetTrajectories targetTrajectories({0.0}, {x}, {u});
    referenceManagerPtr =
        std::make_shared<ReferenceManager>(std::move(targetTrajectories));
    solver->setReferenceManager(referenceManagerPtr);
  }

  // Setup loop
  double simDt = 0.005;
  double horizonDuration = 4.0;

  double currentTime = 0.0;
  vector_t currentState = (vector_t(STATE_DIM) << 0.0, 1.0).finished();
  PrimalSolution primalSolution;
  auto dynamicsDiscretizer =
      selectDynamicsDiscretization(SensitivityIntegratorType::EULER);

  std::string filePath = "/tmp/SampleOscillator.txt";
  std::ofstream ofs(filePath);
  ofs << "time x[0] x[1] u[0]" << std::endl;

  // Control loop
  while (currentTime < 10.0) {
    // Solve MPC
    double finalTime = currentTime + horizonDuration;
    if (currentTime == 0.0) {
      solver->run(currentTime, currentState, finalTime);
    } else {
      solver->run(currentTime, currentState, finalTime, primalSolution);
    }
    primalSolution = solver->primalSolution(finalTime);
    vector_t currentInput = primalSolution.inputTrajectory_[0];

    // Dump log
    ofs << currentTime << " " << currentState.transpose() << " "
        << currentInput.transpose() << std::endl;

    // Simulate
    currentState = dynamicsDiscretizer(*problem.dynamicsPtr, currentTime,
                                       currentState, currentInput, simDt);
    currentTime += simDt;
  }

  std::cout << "Run the following commands in gnuplot:\n"
            << "  set key autotitle columnhead\n"
            << "  set key noenhanced\n"
            << "  plot \"" << filePath
            << "\" u 1:2 w lp, \"\" u 1:3 w lp, \"\" u 1:4 w lp\n";

  return 0;
}
