
// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#include "frenet_optimal_trajectory.h"

#include "ros/ros.h"

namespace shenlan {
#define MAX_SPEED 50.0 / 3.6     // maximum speed [m/s]
#define MAX_ACCEL 2.0            // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0        // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0       // maximum road width [m]
#define D_ROAD_W 1.0             // road width sampling length [m]
#define DT 0.2                   // time tick [s]
#define MAXT 5.0                 // max prediction time [m]
#define MINT 4.0                 // min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6  // target speed [m/s]
#define D_T_S 5.0 / 3.6          // target speed sampling length [m/s]
#define N_S_SAMPLE 1             // sampling number of target speed
#define ROBOT_RADIUS 1.5         // robot radius [m]

#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

FrenetOptimalTrajectory::FrenetOptimalTrajectory() {}
FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {}

float FrenetOptimalTrajectory::sum_of_power(std::vector<float> value_list) {
  float sum = 0;
  for (float item : value_list) {
    sum += item * item;
  }
  return sum;
};


// c_d initial lateral offset, c_d_d initial lateral speed, c_d_dd, initial lateral acc
Vec_Path FrenetOptimalTrajectory::calc_frenet_paths(float c_speed, float c_d,
                                                    float c_d_d, float c_d_dd,
                                                    float s0) {
  std::vector<FrenetPath> fp_list;
 
  //lateral sampling
  for (float l = -MAX_ROAD_WIDTH; l <= MAX_ROAD_WIDTH; l += D_ROAD_W) {
    
    for (float t = MINT; t <= MAXT; t += DT) {
      FrenetPath fr;

      QuinticPolynomial qui(c_d, c_d_d, c_d_dd, l, 0.0, 0.0, t);
     
      
      for (float ti = 0.0; ti <= t + 0.01; ti += DT) {
        
        fr.t.push_back(ti);
        fr.d.push_back(qui.calc_point(ti));
        fr.d_d.push_back(qui.calc_first_derivative(ti));
        fr.d_dd.push_back(qui.calc_second_derivative(ti));
        fr.d_ddd.push_back(qui.calc_third_derivative(ti));
      }

      // longitude sampling

      for (float v = TARGET_SPEED - D_T_S * N_S_SAMPLE; v <= TARGET_SPEED + D_T_S * N_S_SAMPLE; v += D_T_S) {
        FrenetPath tfp  = fr;
        QuarticPolynomial qua(s0, c_speed, 0.0, v, 0.0, t);

        float J_lat = 0.0;
        float J_lon = 0.0;
        for (size_t i = 0; i < tfp.t.size(); ++i) {
          tfp.s.push_back(qua.calc_point(tfp.t[i]));
          tfp.s_d.push_back(qua.calc_first_derivative(tfp.t[i]));
          tfp.s_dd.push_back(qua.calc_second_derivative(tfp.t[i]));
          tfp.s_ddd.push_back(qua.calc_third_derivative(tfp.t[i]));

          J_lat += std::pow(tfp.d_ddd[i], 2);
          J_lon += std::pow(tfp.s_ddd[i], 2);
        }

        float v_err = std::pow((tfp.s_d.back() - TARGET_SPEED), 2);

        //lateral cost
        tfp.cd = KJ * J_lat + KT * t + KD * std::pow(tfp.d.back(), 2);

        //longitude cost
        tfp.cv = KJ * J_lon + KT * t + KD * v_err;

        //total cost of a 2d Traj

        tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;
        
        
        fp_list.push_back(tfp);
      }

    }
  }
  
  return fp_list;
};


void FrenetOptimalTrajectory::calc_global_paths(Vec_Path& path_list,
                                                Spline2D csp){

    
    for (size_t i = 0; i < path_list.size(); ++i) {
      FrenetPath fp = path_list[i];

      for (size_t j = 0; j < fp.s.size(); ++j) {

        float s = fp.s[j];
        float xr = csp.calc_postion(s)[0];
        float yr = csp.calc_postion(s)[1];
        float theta_r = csp.calc_yaw(s);
        float k_r = csp.calc_curvature(s);

        float l_pie = fp.d_d[j] / (fp.s_d[j]);
        float l_pie2 = (fp.d_dd[j] - l_pie * fp.s_dd[j]) / (std::pow(fp.s_d[j], 2));

        float x = xr - fp.d[j] * std::sin(theta_r);
        float y = yr + fp.d[j] * std::cos(theta_r);
        float theta = std::atan2(l_pie, 1 - k_r * fp.d[j]) + theta_r;
        float ds2 = std::pow(fp.d_d[j], 2) * std::pow(1 - k_r * fp.d[j], 2) + std::pow(fp.d_d[j], 2);
        float ds = std::sqrt(ds2);

        if (theta > M_PI) {
          theta -= 2 * M_PI;
        } else if (theta < -M_PI) {
          theta += 2 * M_PI;
        }

        float delta_theta = theta - theta_r;
        const float cos_delta_theta = std::cos(delta_theta);
        const float tan_delta_theta = std::tan(delta_theta);
        const float one_minus_kr_l = 1 - k_r * fp.d[j];


        const float k_x_left = ((l_pie2 + k_r * l_pie * tan_delta_theta)) * std::pow(cos_delta_theta, 2) / one_minus_kr_l + k_r;
        const float k_x_right = cos_delta_theta / one_minus_kr_l;
        const float k_x = k_x_left * k_x_right;
        // std::cout << "curvature is :" << k_x << std::endl;
        
        path_list[i].x.push_back(x);
        path_list[i].y.push_back(y);
        path_list[i].yaw.push_back(theta);
        path_list[i].ds.push_back(ds);
        path_list[i].c.push_back(k_x);
      }

      
    }
    
};

bool FrenetOptimalTrajectory::check_collision(FrenetPath path,
                                              const Vec_Poi ob) {
  for (auto point : ob) {
    for (unsigned int i = 0; i < path.x.size(); i++) {
      float dist = std::pow((path.x[i] - point[0]), 2) +
                   std::pow((path.y[i] - point[1]), 2);
      if (dist <= ROBOT_RADIUS * ROBOT_RADIUS) {
        return false;
      }
    }
  }
  return true;
};


Vec_Path FrenetOptimalTrajectory::check_paths(Vec_Path path_list,
                                              const Vec_Poi ob) {
  Vec_Path output_fp_list;
  for (size_t i = 0; i < path_list.size(); ++i) {
    FrenetPath fp = path_list[i];
    float max_v = *max_element(fp.s_d.begin(), fp.s_d.end());
    if (abs(max_v) > MAX_SPEED) continue;
    
    if (!check_collision(fp, ob)) continue;

    float max_acc = *max_element(fp.s_dd.begin(), fp.s_dd.end());
    if (abs(max_acc) > MAX_ACCEL) continue;

    if (fp.c.empty()) {
      std::cout << "No curvature data!" << std::endl;
      continue;
    }
    float max_c = *max_element(fp.c.begin(), fp.c.end());
    if (abs(max_c) > MAX_CURVATURE) continue;

    output_fp_list.push_back(fp);
  }
  std::cout << "the numbers of candidate trajectory is :" << output_fp_list.size() <<std::endl;
  feasible_traj_ = output_fp_list;
  return output_fp_list;
};

FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
    Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd,
    Vec_Poi ob) {
  
  std::cout << "frenet_optimal_planning module is working!" << std::endl;
  // sampling longitude and lateral trajectory respectively
  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  
  std::cout << "sampling module finished" << std::endl;
  // based on refference line, calculate x, y, yaw, ds, c of the trajectory
  calc_global_paths(fp_list, csp);
  std::cout << "calc_global_path finished" << std::endl;
  
  // trajectory check, including dynamic feasible and collision
  Vec_Path save_paths = check_paths(fp_list, ob);
  std::cout << "check module finished" << std::endl;
  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for (auto path : save_paths) {
    if (min_cost >= path.cf) {
      min_cost = path.cf;
      final_path = path;
    }
  }

  return final_path;
};

FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
    Spline2D csp, const FrenetInitialConditions& frenet_init_conditions,
    Vec_Poi ob) {
  float c_speed = frenet_init_conditions.c_speed;
  float c_d = frenet_init_conditions.c_d;
  float c_d_d = frenet_init_conditions.c_d_d;
  float c_d_dd = frenet_init_conditions.c_d_dd;
  float s0 = frenet_init_conditions.s0;

  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fp_list, csp);
  Vec_Path save_paths = check_paths(fp_list, ob);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for (auto path : save_paths) {
    if (min_cost >= path.cf) {
      min_cost = path.cf;
      final_path = path;
    }
  }
  return final_path;
}

}  // namespace shenlan
