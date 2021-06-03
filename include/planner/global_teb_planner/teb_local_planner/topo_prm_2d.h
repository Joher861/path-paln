

#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H

#include "teb_local_planner/raycast_2d.h"
#include "teb_local_planner/teb_log.h"
#include "local_map/local_map.h"
#include "misc/planning_typedefs.h"

#include <random>
#include <vector>

using namespace std;
using namespace planning_map;

namespace planning_planner {

/* ---------- used for iterating all topo combination ---------- */
class TopoIterator {
private:
  /* data */
  vector<int> path_nums_;
  vector<int> cur_index_;
  int combine_num_;
  int cur_num_;

  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  TopoIterator(vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    TEB_DEBUG_LOG("[Topo]: merged path num: %d" ,combine_num_);
  }
  TopoIterator() {
  }
  ~TopoIterator() {
  }

  bool nextIndex(vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
class GraphNode {
private:
  /* data */

public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode(/* args */) {
  }
  GraphNode(Eigen::Vector2d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {
  }

  vector<shared_ptr<GraphNode>> neighbors_;
  Eigen::Vector2d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

  typedef shared_ptr<GraphNode> Ptr;
};

class TopologyPRM {
private:
  /* data */
  SDFMap::Ptr sdf_map_;  // environment representation

  // sampling generator
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d sample_r_;
  Eigen::Vector2d translation_;
  Eigen::Matrix3d rotation_;


  // roadmap data structure, 0:start, 1:goal, 2-n: others
  list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector2d>> raw_paths_;
  vector<vector<Eigen::Vector2d>> short_paths_;
  vector<vector<Eigen::Vector2d>> final_paths_;
  vector<Eigen::Vector2d> start_pts_, end_pts_;

  // raycasting
  vector<RayCaster> casters_;
  Eigen::Vector3d offset_;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_, max_raw_path2_;
  int short_cut_num_;
  Eigen::Vector3d sample_inflate_;
  double resolution_;

  double ratio_to_short_;
  int reserve_num_;

  bool parallel_shortcut_;

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  list<GraphNode::Ptr> createGraph(Eigen::Vector2d start, Eigen::Vector2d end);
  vector<vector<Eigen::Vector2d>> searchPaths();
  void shortcutPaths();
  vector<vector<Eigen::Vector2d>> pruneEquivalent(vector<vector<Eigen::Vector2d>>& paths);
  vector<vector<Eigen::Vector2d>> selectShortPaths(vector<vector<Eigen::Vector2d>>& paths, int step);

  /* ---------- helper ---------- */
  inline Eigen::Vector2d getSample();
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector2d pt);  // find pairs of visibile guard
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                      Eigen::Vector2d pt);  // test redundancy with existing
                                            // connection between two guard
  bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double thresh,
                 Eigen::Vector2d& pc, int caster_id = 0);
  bool triangleVisib(Eigen::Vector2d pt, Eigen::Vector2d p1, Eigen::Vector2d p2);
  void pruneGraph();

  void depthFirstSearch(vector<GraphNode::Ptr>& vis);

  vector<Eigen::Vector2d> discretizeLine(Eigen::Vector2d p1, Eigen::Vector2d p2);
  vector<vector<Eigen::Vector2d>> discretizePaths(vector<vector<Eigen::Vector2d>>& path);

  vector<Eigen::Vector2d> discretizePath(vector<Eigen::Vector2d> path);
  void shortcutPath(vector<Eigen::Vector2d> path, int path_id, int iter_num = 1);

  vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path, int pt_num);
  Eigen::Vector2d getOrthoPoint(const vector<Eigen::Vector2d>& path);

  int shortestPath(vector<vector<Eigen::Vector2d>>& paths);

  double clearance_;
public:
  bool sameTopoPath(const vector<Eigen::Vector2d>& path1, const vector<Eigen::Vector2d>& path2,
                    double thresh);

  TopologyPRM(/* args */);
  ~TopologyPRM();

  void init(double obstacle_dis, int path_num);

  void setEnvironment(const SDFMap::Ptr& env);

  void findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end, vector<Eigen::Vector2d> start_pts,
                     vector<Eigen::Vector2d> end_pts, list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector2d>>& raw_paths,
                     vector<vector<Eigen::Vector2d>>& filtered_paths,
                     vector<vector<Eigen::Vector2d>>& select_paths);

  void findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end,
                                  vector<vector<Eigen::Vector2d>>& select_paths);
  double pathLength(const vector<Eigen::Vector2d>& path);
  vector<Eigen::Vector2d> pathToGuidePts(vector<Eigen::Vector2d>& path, int pt_num);

};

}  // namespace fast_planner

#endif