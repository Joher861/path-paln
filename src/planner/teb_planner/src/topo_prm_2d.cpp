

#include "teb_local_planner/topo_prm_2d.h"
#include "timer/timer.h"
#include <thread>

using namespace planning_utils;

namespace planning_planner {

TopologyPRM::TopologyPRM() {}

TopologyPRM::~TopologyPRM() {}

void TopologyPRM::init(double obstacle_dis, int path_num) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // init parameter
  sample_inflate_(0) = 1;
  sample_inflate_(1) = 3.5;
  sample_inflate_(2) = 1;

  clearance_ = obstacle_dis; //与障碍物的间隔
  short_cut_num_ = 1;
  reserve_num_ = path_num;
  ratio_to_short_ = 5.5;
  max_sample_num_ = 2000;
  max_sample_time_ = 0.005;
  max_raw_path_ = 300;  //总共搜索多少条路径
  max_raw_path2_ = 25; //选取节点数最少的前多少个路径
  parallel_shortcut_ = true;
  resolution_ = sdf_map_->getResolution();
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - sdf_map_->getOrigin() / resolution_;

  for (int i = 0; i < max_raw_path_; ++i) {
    casters_.push_back(RayCaster());
  }
}

void TopologyPRM::findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end,
                                vector<Eigen::Vector2d> start_pts, vector<Eigen::Vector2d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector2d>>& raw_paths,
                                vector<vector<Eigen::Vector2d>>& filtered_paths,
                                vector<vector<Eigen::Vector2d>>& select_paths) {
  uint64_t t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  /* ---------- create the topo graph ---------- */
  t1 = Timer::getSystemTimestampMS();

  start_pts_ = start_pts;
  end_pts_ = end_pts;

  graph = createGraph(start, end);

  graph_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  /* ---------- search paths in the graph ---------- */
  t1 = Timer::getSystemTimestampMS();

  raw_paths = searchPaths();

  search_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  /* ---------- path shortening ---------- */
  // for parallel, save result in short_paths_
  t1 = Timer::getSystemTimestampMS();

  shortcutPaths();

  short_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  /* ---------- prune equivalent paths ---------- */
  t1 = Timer::getSystemTimestampMS();

  filtered_paths = pruneEquivalent(short_paths_);

  prune_time = (Timer::getSystemTimestampMS() - t1) * 0.001;
  // TEB_DEBUG_LOG("prune: %lf",prune_time);

  /* ---------- select N shortest paths ---------- */
  t1 = Timer::getSystemTimestampMS();

  select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  final_paths_ = select_paths;

  double total_time = graph_time + search_time + short_time + prune_time + select_time;

  TEB_DEBUG_LOG("\n[Topo]: total time: %lf, graph: %lf, search: %lf, short: %lf, prune: %lf, select: %lf" ,
      total_time,graph_time,search_time,short_time,prune_time,select_time);
}


void TopologyPRM::findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end,
                                vector<vector<Eigen::Vector2d>>& select_paths) {
  list<GraphNode::Ptr> graph;
  vector<vector<Eigen::Vector2d>> raw_paths;
  vector<vector<Eigen::Vector2d>> filtered_paths;

  uint64_t t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  /* ---------- create the topo graph ---------- */
  t1 = Timer::getSystemTimestampMS();

  start_pts_.clear();
  end_pts_.clear();

  graph = createGraph(start, end);

  graph_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  /* ---------- search paths in the graph ---------- */
  t1 = Timer::getSystemTimestampMS();

  raw_paths = searchPaths();

  search_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  /* ---------- path shortening ---------- */
  // for parallel, save result in short_paths_
  t1 = Timer::getSystemTimestampMS();

  shortcutPaths();

  short_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  /* ---------- prune equivalent paths ---------- */
  t1 = Timer::getSystemTimestampMS();

  filtered_paths = pruneEquivalent(short_paths_);

  prune_time = (Timer::getSystemTimestampMS() - t1) * 0.001;
  // TEB_DEBUG_LOG("prune: %ld" ,(t2 - t1) * 0.001);

  /* ---------- select N shortest paths ---------- */
  t1 = Timer::getSystemTimestampMS();

  select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (Timer::getSystemTimestampMS() - t1) * 0.001;

  final_paths_ = select_paths;

  double total_time = graph_time + search_time + short_time + prune_time + select_time;

  TEB_DEBUG_LOG("\n[Topo]: total time: %lf, graph: %lf, search: %lf, short: %lf, prune: %lf, select: %lf" ,
      total_time,graph_time,search_time,short_time,prune_time,select_time);
}


list<GraphNode::Ptr> TopologyPRM::createGraph(Eigen::Vector2d start, Eigen::Vector2d end) {
  TEB_DEBUG_LOG("[Topo]: searching----------------------" );

  /* init the start, end and sample region */
  graph_.clear();
  // collis_.clear();

  GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // sample region
  sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0);
  sample_r_(1) = sample_inflate_(1);
  sample_r_(2) = sample_inflate_(2);

  // transformation
  translation_ = 0.5 * (start + end);

  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  Eigen::Vector3d tmp_vector;
  Eigen::Vector2d tmp_vector1 = end - translation_;
  tmp_vector(0) = tmp_vector1(0);
  tmp_vector(1) = tmp_vector1(1);
  tmp_vector(2) = 0;

  xtf = tmp_vector.normalized();
  ytf = xtf.cross(downward).normalized();
  ztf = xtf.cross(ytf);

  rotation_.col(0) = xtf;
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  int node_id = 1;

  /* ---------- main loop ---------- */
  int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector2d pt;
  uint64_t t1, t2;
  while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
    t1 = Timer::getSystemTimestampMS();

    pt = getSample();
    ++sample_num;
    double dist;
    // Eigen::Vector2d grad;
    // edt_environment_->evaluateEDTWithGrad(pt, -1.0, dist, grad);
    dist = sdf_map_->getDistance(pt);
    if (dist <= clearance_) {
      sample_time += (Timer::getSystemTimestampMS() - t1) * 0.001;
      continue;
    }

    /* find visible guard */
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    if (visib_guards.size() == 0) {
      GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    } else if (visib_guards.size() == 2) {
      /* try adding new connection between two guard */
      // vector<pair<GraphNode::Ptr, GraphNode::Ptr>> sort_guards =
      // sortVisibGuard(visib_guards);
      bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {
        sample_time += (Timer::getSystemTimestampMS() - t1) * 0.001;
        continue;
      }
      // new useful connection needed, add new connector
      GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
      graph_.push_back(connector);

      // connect guards
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);

      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
    }

    sample_time += (Timer::getSystemTimestampMS() - t1) * 0.001;
  }

  /* print record */
  TEB_DEBUG_LOG("[Topo]: sample num: %d",sample_num);

  pruneGraph();
  TEB_DEBUG_LOG("[Topo]: node num: %d",graph_.size());

  return graph_;
  // return searchPaths(start_node, end_node);
}

vector<GraphNode::Ptr> TopologyPRM::findVisibGuard(Eigen::Vector2d pt) {
  vector<GraphNode::Ptr> visib_guards;
  Eigen::Vector2d pc;
  int visib_num = 0;

  /* find visible GUARD from pt */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) continue;

    if (lineVisib(pt, (*iter)->pos_, resolution_,pc)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2) break;
    }
  }

  return visib_guards;
}

bool TopologyPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector2d pt) {
  vector<Eigen::Vector2d> path1(3), path2(3);
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  vector<Eigen::Vector2d> connect_pts;
  bool has_connect = false;
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        path2[1] = g1->neighbors_[i]->pos_;
        bool same_topo = sameTopoPath(path1, path2, 0.0);
        if (same_topo) {
          // get shorter connection ?
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;
            // ROS_WARN("shorter!");
          }
          return false;
        }
      }
    }
  }
  return true;
}

Eigen::Vector2d TopologyPRM::getSample() {
  /* sampling */
  Eigen::Vector3d pt;
  Eigen::Vector3d tmp;
  Eigen::Vector2d pt_r;
  pt(0) = rand_pos_(eng_) * sample_r_(0);
  pt(1) = rand_pos_(eng_) * sample_r_(1);
  pt(2) = rand_pos_(eng_) * sample_r_(2);

  tmp(0) = translation_(0);
  tmp(1) = translation_(1);
  tmp(0) = 0;

  pt = rotation_ * pt + tmp;
  pt_r(0) = pt(0);
  pt_r(1) = pt(1);
  
  TEB_DEBUG_LOG("sample{%f,%f}",pt_r(0),pt_r(1));
  return pt_r;
}
//test if line g1g2 is in collision
bool TopologyPRM::lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double thresh,
                            Eigen::Vector2d& pc, int caster_id) {
  Eigen::Vector2d ray_pt;
  Eigen::Vector3i pt_id;
  Eigen::Vector3d pc_3d;

  double dist;

  casters_[caster_id].setInput(p1 / resolution_, p2 / resolution_);
  while (casters_[caster_id].step(ray_pt)) {
    pt_id(0) = ray_pt(0) + offset_(0);
    pt_id(1) = ray_pt(1) + offset_(1);
    pt_id(2) = 0;
    dist = sdf_map_->getDistance(pt_id);//这里传入的是栅格坐标
    if (dist <= thresh) {
      sdf_map_->indexToPos(pt_id, pc_3d);
      pc(0) = pc_3d(0);
      pc(1) = pc_3d(1);

      return false;
    }
  }
  return true;
}

void TopologyPRM::pruneGraph() {
  /* prune useless node */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1) continue;

      /* core */
      // std::cout << "id: " << (*iter1)->id_ << std::endl;
      if ((*iter1)->neighbors_.size() <= 1) {   //把只有单独一个连接的点去除掉
        // delete this node from others' neighbor
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

vector<vector<Eigen::Vector2d>> TopologyPRM::pruneEquivalent(vector<vector<Eigen::Vector2d>>& paths) {
  vector<vector<Eigen::Vector2d>> pruned_paths;
  if (paths.size() < 1) return pruned_paths;

  /* ---------- prune topo equivalent path ---------- */
  // output: pruned_paths
  vector<int> exist_paths_id;
  exist_paths_id.push_back(0);

  for (int i = 1; i < paths.size(); ++i) {
    // compare with exsit paths
    bool new_path = true;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // compare with one path
      bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0);

      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      exist_paths_id.push_back(i);
    }
  }

  // save pruned paths
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  TEB_DEBUG_LOG("pruned path num: %d" , pruned_paths.size());

  return pruned_paths;
}

vector<vector<Eigen::Vector2d>> TopologyPRM::selectShortPaths(vector<vector<Eigen::Vector2d>>& paths,
                                                              int step) {
  /* ---------- only reserve top short path ---------- */
  vector<vector<Eigen::Vector2d>> short_paths;
  vector<Eigen::Vector2d> short_path;
  double min_len;

  for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i) {
    int path_id = shortestPath(paths);
    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = pathLength(paths[path_id]);
      paths.erase(paths.begin() + path_id);
    } else {
      double rat = pathLength(paths[path_id]) / min_len;
      if (rat < ratio_to_short_) {
        short_paths.push_back(paths[path_id]);
        paths.erase(paths.begin() + path_id);
      } else {
        break;
      }
    }
  }
  TEB_DEBUG_LOG("select path num: %d" ,short_paths.size());

  /* ---------- merge with start and end segment ---------- */
  for (int i = 0; i < short_paths.size(); ++i) {
    if(!start_pts_.empty())
      short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());
    
    if(!end_pts_.empty())
      short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());
  }
  for (int i = 0; i < short_paths.size(); ++i) {
    shortcutPath(short_paths[i], i, 5);
    short_paths[i] = short_paths_[i];
  }

  short_paths = pruneEquivalent(short_paths);

  return short_paths;
}
//对应论文中的Fig. 6. 将两段轨迹分成多个点，两点连成线，然后测试是否与障碍物干涉
bool TopologyPRM::sameTopoPath(const vector<Eigen::Vector2d>& path1,
                               const vector<Eigen::Vector2d>& path2, double thresh) {
  // calc the length
  double len1 = pathLength(path1);
  double len2 = pathLength(path2);

  double max_len = max(len1, len2);

  int pt_num = ceil(max_len / resolution_);

  TEB_DEBUG_LOG("pt num: %d" ,pt_num);

  vector<Eigen::Vector2d> pts1 = discretizePath(path1, pt_num);
  vector<Eigen::Vector2d> pts2 = discretizePath(path2, pt_num);

  Eigen::Vector2d pc;
  for (int i = 0; i < pt_num; ++i) {
    if (!lineVisib(pts1[i], pts2[i], thresh,pc)) {
      return false;
    }
  }

  return true;
}

int TopologyPRM::shortestPath(vector<vector<Eigen::Vector2d>>& paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}
double TopologyPRM::pathLength(const vector<Eigen::Vector2d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

vector<Eigen::Vector2d> TopologyPRM::discretizePath(const vector<Eigen::Vector2d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector2d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector2d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

vector<Eigen::Vector2d> TopologyPRM::pathToGuidePts(vector<Eigen::Vector2d>& path, int pt_num) {
  return discretizePath(path, pt_num);
}

void TopologyPRM::shortcutPath(vector<Eigen::Vector2d> path, int path_id, int iter_num) {
  vector<Eigen::Vector2d> short_path = path;
  vector<Eigen::Vector2d> last_path;

  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

    vector<Eigen::Vector2d> dis_path = discretizePath(short_path);

    if (dis_path.size() < 2) {
      short_paths_[path_id] = dis_path;
      return;
    }

    /* visibility path shortening */
    Eigen::Vector2d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());
    for (int i = 1; i < dis_path.size(); ++i) {
      if (lineVisib(short_path.back(), dis_path[i], resolution_, colli_pt, path_id)) continue;

      dist = sdf_map_->getDistWithGradTrilinear(colli_pt, grad);
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        colli_pt = colli_pt + resolution_ * push_dir;
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());

    /* break if no shortcut */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if (len2 > len1) {
      // ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k +
      // 1);
      short_path = last_path;
      break;
    }
  }

  short_paths_[path_id] = short_path;
}

void TopologyPRM::shortcutPaths() {
  short_paths_.resize(raw_paths_.size());

  if (parallel_shortcut_) {
    vector<thread> short_threads;
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads.push_back(thread(&TopologyPRM::shortcutPath, this, raw_paths_[i], i, 1));
    }
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads[i].join();
    }
  } else {
    for (int i = 0; i < raw_paths_.size(); ++i) shortcutPath(raw_paths_[i], i);
  }
}

vector<Eigen::Vector2d> TopologyPRM::discretizeLine(Eigen::Vector2d p1, Eigen::Vector2d p2) {
  Eigen::Vector2d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);

  vector<Eigen::Vector2d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  for (int i = 0; i <= seg_num; ++i) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

vector<Eigen::Vector2d> TopologyPRM::discretizePath(vector<Eigen::Vector2d> path) {
  vector<Eigen::Vector2d> dis_path, segment;

  if (path.size() < 2) {
    TEB_ERROR_LOG("what path? ");
    return dis_path;
  }

  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine(path[i], path[i + 1]);

    if (segment.size() < 1) continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    if (i != path.size() - 2) dis_path.pop_back();
  }
  return dis_path;
}

vector<vector<Eigen::Vector2d>> TopologyPRM::discretizePaths(vector<vector<Eigen::Vector2d>>& path) {
  vector<vector<Eigen::Vector2d>> dis_paths;
  vector<Eigen::Vector2d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.push_back(dis_path);
  }

  return dis_paths;
}

Eigen::Vector2d TopologyPRM::getOrthoPoint(const vector<Eigen::Vector2d>& path) {
  Eigen::Vector2d x1 = path.front();
  Eigen::Vector2d x2 = path.back();

  Eigen::Vector2d dir = (x2 - x1).normalized();
  Eigen::Vector2d mid = 0.5 * (x1 + x2);

  double min_cos = 1000.0;
  Eigen::Vector2d pdir;
  Eigen::Vector2d ortho_pt;

  for (int i = 1; i < path.size() - 1; ++i) {
    pdir = (path[i] - mid).normalized();
    double cos = fabs(pdir.dot(dir));

    if (cos < min_cos) {
      min_cos = cos;
      ortho_pt = path[i];
    }
  }

  return ortho_pt;
}

// search for useful path in the topo graph by DFS
vector<vector<Eigen::Vector2d>> TopologyPRM::searchPaths() {
  raw_paths_.clear();

  vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());

  depthFirstSearch(visited);

  // sort the path by node number
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num) max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num) min_node_num = raw_paths_[i].size();
    path_list[int(raw_paths_[i].size())].push_back(i);
  }

  // select paths with less nodes
  vector<vector<Eigen::Vector2d>> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path2_) {  //选max_raw_path2_个节点最少的路径
        reach_max = true;
        break;
      }
    }
    if (reach_max) break;
  }
  TEB_DEBUG_LOG("raw path num: %d  filter_raw_paths_num: %d" ,raw_paths_.size(),filter_raw_paths.size());

  raw_paths_ = filter_raw_paths;//搜索出来并保留的节点数靠前的路径

  return raw_paths_;
}

void TopologyPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
  GraphNode::Ptr cur = vis.back();

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // check reach goal
    if (cur->neighbors_[i]->id_ == 1) { //goal点的id就是1
      // add this path to paths set
      vector<Eigen::Vector2d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_) return;     //最多搜索出max_raw_path_条轨迹

      break;
    }
  }

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // skip reach goal
    if (cur->neighbors_[i]->id_ == 1) continue;

    // skip already visited node
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit) continue;

    // recursive search
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_) return;

    vis.pop_back();
  }
}

void TopologyPRM::setEnvironment(const SDFMap::Ptr& env) { this->sdf_map_ = env; }


// TopologyPRM::
}  // namespace fast_planner