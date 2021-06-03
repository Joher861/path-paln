#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "hybrid_breadth_first.h"

using namespace std;

float wrapMax(float x, float max)
{
  /* integer math: `(max + x % max) % max` */
  return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
float wrapMinMax(float x, float min, float max)
{
  return min + wrapMax(x - min, max - min);
}

float wrapAngle(float yaw)
{
  return wrapMinMax(yaw, -M_PI, M_PI);
}

HBF::HBF()
{
}

HBF::~HBF() {}

bool HBF::compare_maze_s(const HBF::maze_s &lhs, const HBF::maze_s &rhs)
{
  return lhs.f < rhs.f;
}

// 距离目标点的距离作为启发式的cost
double HBF::heuristic(double x, double y, vector<double> goal)
{
  double x_diff = abs(x - goal[0]);
  double y_diff = abs(y - goal[1]);
  double index_diff = hypot(x_diff, y_diff);
  // return fabs(y - goal[0]) + fabs(x - goal[1]); //return grid distance to goal
  return index_diff;
}

int HBF::theta_to_stack_number(double theta)
{
  /*
  Takes an angle (in radians) and returns which "stack" in the 3D configuration space
  this angle corresponds to. Angles near 0 go in the lower stacks while angles near 
  2 * pi go in the higher stacks.
  */
  double new_theta = fmod((theta + 2 * M_PI), (2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2 * M_PI))) % NUM_THETA_CELLS;
  return stack_number;
}

int HBF::idx(double float_num)
{
  /*
  Returns the index into the grid for continuous position. So if x is 3.621, then this
  would return 3 to indicate that 3.621 corresponds to array index 3.
  */
  return int(floor(float_num));
}

// 根据自行车的运动学模型,进行扩展
vector<HBF::maze_s> HBF::expand(HBF::maze_s state, vector<double> goal, float speed)
{
  double g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;

  double g2;
  vector<HBF::maze_s> next_states;
  // 角度的限制
  for (double delta_i = -75; delta_i <= 75; delta_i += 2)
  {
    double delta = M_PI / 180.0 * delta_i;
    double omega = speed / LENGTH * tan(delta);

    if (omega > M_PI)
      omega = M_PI;
    else if (omega < -M_PI)
      omega = -M_PI;

    double theta2 = theta + omega;

    if (theta2 < -M_PI)
    {
      theta2 += 2 * M_PI;
    }
    if (theta2 > M_PI)
    {
      theta2 -= 2 * M_PI;
    }

    double x2 = x + SPEED * cos(theta2);
    double y2 = y + SPEED * sin(theta2);
    HBF::maze_s state2;

    float a = 0;
    if (abs(delta) < M_PI / 4)
      a = 0;
    else if (abs(delta) < M_PI / 3)
      a = 3;
    else
      a = 5;

    g2 = a + SPEED + g;
    // g2 = a + SPEED;

    state2.f = g2 + heuristic(x2, y2, goal);
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);
  }

  return next_states;
}

vector<HBF::maze_s> HBF::reconstruct_path(vector<vector<vector<HBF::maze_s>>> came_from,
                                          vector<double> start, vector<double> goal, HBF::maze_s final,
                                          float gridOriginX, float gridOriginY, float AStarPrecision)
{

  vector<maze_s> path = {final};
  int stack = theta_to_stack_number(final.theta);

  maze_s current = came_from[stack][idx(final.x)][idx(final.y)];

  stack = theta_to_stack_number(current.theta);

  double x = current.x;
  double y = current.y;

  while (x != start[0] || y != start[1])
  {
    path.push_back(current);
    current = came_from[stack][idx(x)][idx(y)];
    x = current.x;
    y = current.y;
    stack = theta_to_stack_number(current.theta);
  }

  // 加入起点
  maze_s start_point;
  start_point.x = start[0] * AStarPrecision + gridOriginX;
  start_point.y = start[1] * AStarPrecision + gridOriginY;
  start_point.theta = start[2];

  vector<maze_s> path_res;
  path_res.push_back(start_point);

  // 倒序加入
  for (int i = path.size() - 1; i >= 0; i--)
  {
    maze_s pathTemp = path[i];
    pathTemp.x = path[i].x * AStarPrecision + gridOriginX;
    pathTemp.y = path[i].y * AStarPrecision + gridOriginY;
    pathTemp.theta = pathTemp.theta;
    path_res.push_back(pathTemp);
  }
  path_res.pop_back();
  // 加入终点
  maze_s goal_point;
  goal_point.x = goal[0] * AStarPrecision + gridOriginX;
  goal_point.y = goal[1] * AStarPrecision + gridOriginY;
  goal_point.theta = path_res.back().theta; // 不考虑goal_point的角度

  path_res.push_back(goal_point);

  for (int i = 0; i < (int)path_res.size(); i++)
  {
    int index = min(i + 10, (int)path_res.size()-1);
    path_res[i].theta = atan2(path_res[index].y - path_res[i].y, path_res[index].x - path_res[i].x);
  }

  return path_res;
}

HBF::maze_path HBF::search(vector<vector<int>> grid, vector<double> start, vector<double> goal, float AStarPrecision, float agvLength, float agvWidth)
{
  /*
  Working Implementation of breadth first search. Does NOT use a heuristic
  and as a result this is pretty inefficient. Try modifying this algorithm 
  into hybrid A* by adding heuristics appropriately.
  */
  int num = max(grid[0].size(), grid.size());
  vector<vector<vector<int>>> closed(NUM_THETA_CELLS, vector<vector<int>>(num, vector<int>(num)));
  // theta,x,y 三个状态量
  // vector<vector<vector<int>>> closed(NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector<vector<vector<maze_s>>> came_from(NUM_THETA_CELLS, vector<vector<maze_s>>(closed[0].size(), vector<maze_s>(closed[0][0].size())));
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  double g = 0;

  maze_s state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];

  // heuristic() return grid distance to goal
  state.f = g + heuristic(state.x, state.y, goal);
  state.theta = theta;

  closed[stack][idx(state.x)][idx(state.y)] = 1;
  came_from[stack][idx(state.x)][idx(state.y)] = state;

  int total_closed = 1;
  vector<maze_s> opened = {state};
  bool finished = false;
  // open_set 和 close_set的搜索框架

  while (!opened.empty())
  {

    // 按照cost最小排序
    //grab first elment  // cost最小
    maze_s current = opened[0];
    opened.erase(opened.begin()); //pop first element
    // 判断和 goal_point的距离,是否到达目标点
    double x = current.x;
    double y = current.y;
    int x_diff = abs(idx(x) - goal[0]);
    int y_diff = abs(idx(y) - goal[1]);
    float index_diff = hypot(x_diff, y_diff);

    if (index_diff < 2)
    {
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;
      return path;
    }

    // 扩展当前节点的周围节点,运动学方程
    float speed = SPEED * AStarPrecision;
    vector<maze_s> next_state = expand(current, goal, speed);

    for (int i = 0; i < (int)next_state.size(); i++)
    {

      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if ((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
      {
        //invalid cell
        continue;
      }

      int stack2 = theta_to_stack_number(theta2);
      // 没有障碍物
      bool no_obstacel_round_robot = true;
      int xGridAgv = ceil(agvLength / AStarPrecision);
      int yGridAgv2 = ceil(agvWidth / AStarPrecision / 2.0);
      if (closed[stack2][idx(x2)][idx(y2)] != 0)
      {
        no_obstacel_round_robot = no_obstacel_round_robot && false;
      }

      if (no_obstacel_round_robot == true)
      {
        for (int m = 0; m <= xGridAgv; m++)
        {
          for (int n = -yGridAgv2; n <= yGridAgv2; n++)
          {
            float x3 = x2 + m * cos(theta2) - n * sin(theta2);
            float y3 = y2 + m * sin(theta2) + n * cos(theta2);

            if (x3 < 0 || x3 >= (int)grid.size() || y3 < 0 || y3 >= (int)grid[0].size())
            {
              no_obstacel_round_robot = no_obstacel_round_robot && false;
              break;
            }
            else if ((grid[idx(x3)][idx(y3)] != 0))
            {
              no_obstacel_round_robot = no_obstacel_round_robot && false;
              break;
            }
          }
          if (no_obstacel_round_robot == false)
            break;
        }
      }

      if (no_obstacel_round_robot)
      {

        if ((int)opened.size() == 0)
        {
          opened.push_back(next_state[i]);
        }
        else
        {
          for (int j = 0; j < (int)opened.size(); j++)
          {
            if (next_state[i].f < opened[j].f)
            {
              opened.insert(opened.begin() + j, next_state[i]);
              break;
            }
            if (j == (int)opened.size() - 1)
            {
              opened.push_back(next_state[i]);
              break;
            }
          }
        }

        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        total_closed += 1;
      }
      else
      {
      }
    }
  }

  HBF::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}

vector<HBF::maze_s> HBF::hybrid_a_star(vector<vector<int>> grid, vector<double> start, vector<double> goal,float AStarPrecision,float agvLength,float agvWidth,float gridOriginX, float gridOriginY)
{
    HBF::maze_path get_path = search(grid, start, goal, AStarPrecision, agvLength, agvWidth);
    vector<HBF::maze_s> show_path = reconstruct_path(get_path.came_from, start, goal,get_path.final, gridOriginX,gridOriginY, AStarPrecision);
    return show_path;
}