#include <ompl/base/spaces.h>
#include <ompl/geometric/planners/rrtstar.h>
#include <ompl/geometric/planning_groups.h>
#include <ompl/geometric/simple_geometric_state_space.h>
#include <ompl/util/random_numbers.h>
#include <iostream>
#include <vector>

using namespace obamapl;

struct CircleObstacle {
  double x, y;
};

bool isStateValid(const base::State* state, const std::vector<CircleObstacle>& obstacles) {
  // Extract coordinates from state
  const auto* space = state->as<base::SE2StateSpace::StateType>();
  double x = space->getX()[0];
  double y = space->getY()[0];

  // Check for collision with any obstacle
  for (const auto& obstacle : obstacles) {
    double distance = sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2));
    if (distance < 1.1) {  // Consider buffer around obstacle
      return false;
    }
  }

  // State is collision-free
  return true;
}

std::vector<std::pair<double, double>> generatePath(double startX, double startY, double goalX, double goalY,
                                                  const std::vector<CircleObstacle>& obstacles) {
  // Create state space
  auto space = std::make_shared<base::SE2StateSpace>();

  // Set bounds
  space->as<base::SE2StateSpace>()->setBounds(-10.0, 10.0, -10.0, 10.0);

  // Define planning problem
  auto planningGroup = std::make_shared<geometric::SimplePlanningGroup>(space);
  auto startState = std::make_shared<base::SE2StateSpace::StateType>();
  startState->setXyz(startX, startY, 0.0);
  auto goalState = std::make_shared<base::SE2StateSpace::StateType>();
  goalState->setXyz(goalX, goalY, 0.0);

  planningGroup->setStartState(startState);
  planningGroup->setGoalState(goalState);

  // Set object update function for obstacle avoidance
  planningGroup->setStateValidityChecker([obstacles](const base::State* state) {
    return isStateValid(state, obstacles);
  });

  // Create RRT* planner
  auto planner = std::make_shared<geometric::RRTstar>(planningGroup);

  // Set RRT* parameters (adjust as needed)
  planner->setGoalRegionRadius(0.5);
  planner->setRange(1.0);

  // Setup OMPL random number generator
  auto rng = std::make_shared<random_numbers::RNG>();

  // Set OMPL problem definition
  planningGroup->setPlanner(planner);
  planningGroup->setup(rng);

  // Solve the motion planning problem
  base::PlannerStatus status = planningGroup->solvePath();

  // Extract path if successful
  if (status == base::PlannerStatus::solved) {
    auto path = planningGroup->getSolutionPath();
    std::vector<std::pair<double, double>> pathPoints;
    path->getStates(pathPoints);
    return pathPoints;
  } else {
    std::cerr << "Failed to find a solution!" << std::endl;
    return {};
  }
}

int main() {
  // Define start and goal positions
  double startX = 0.0, startY = 0.0;
  double goalX = 8.0, goalY = 5.0;

  // Define obstacles (replace with your obstacle data)
  std::vector<CircleObstacle> obstacles = {
      {3.0, 3.0},
      {6.0, 2.0}
  };

  // Generate path using RRT*
  auto pathPoints = generatePath(startX, startY, goalX, goalY, obstacles);

  // Print path points
  if (!pathPoints.empty()) {
    std::cout << "Path points:" << std::endl;
    for (const auto& point : pathPoints) {
      std::cout << "(" << point.first << ", " << point.second << ")"}
      }
}
