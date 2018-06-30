//
// Helper functions for Path Planning demo
//

#ifndef WAYPOINT
#define WAYPOINT

#include <math.h>
#include <vector>
#include <algorithm>
#include "spline.h"

using namespace std;

typedef struct {
  double s, x, y, dx, dy;
} Waypoint;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double xyDistance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<Waypoint> &wpts)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < wpts.size(); i++)
  {
    double map_x = wpts[i].x;
    double map_y = wpts[i].y;
    double dist = xyDistance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<Waypoint> &wpts)
{

  int closestWaypoint = ClosestWaypoint(x,y,wpts);

  double map_x = wpts[closestWaypoint].x;
  double map_y = wpts[closestWaypoint].y;

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == wpts.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<Waypoint> &wpts)
{
  int next_wp = NextWaypoint(x,y, theta, wpts);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = wpts.size()-1;
  }

  double n_x = wpts[next_wp].x-wpts[prev_wp].x;
  double n_y = wpts[next_wp].y-wpts[prev_wp].y;
  double x_x = x - wpts[prev_wp].x;
  double x_y = y - wpts[prev_wp].y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = xyDistance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-wpts[prev_wp].x;
  double center_y = 2000-wpts[prev_wp].y;
  double centerToPos = xyDistance(center_x,center_y,x_x,x_y);
  double centerToRef = xyDistance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = wpts[0].s;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += xyDistance(wpts[i].x,wpts[i].y,wpts[i+1].x,wpts[i+1].y);
  }

  frenet_s += xyDistance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<Waypoint> &wpts)
{
  int prev_wp = -1;

  while(s > wpts[prev_wp+1].s && (prev_wp < (int)(wpts.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%wpts.size();

  double heading = atan2((wpts[wp2].y-wpts[prev_wp].y),(wpts[wp2].x-wpts[prev_wp].x));
  // the x,y,s along the segment
  double seg_s = (s-wpts[prev_wp].s);

  double seg_x = wpts[prev_wp].x+seg_s*cos(heading);
  double seg_y = wpts[prev_wp].y+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

/* UNUSED
// given a set of waypoints, return a set of interpolated waypoints at every distance dist
vector<Waypoint> getInterpolatedWaypoints(const vector<Waypoint> &wpts, double max_s, double dist)
{
  vector<Waypoint> intp_wpts;

  // extract individual vectors for each element of Waypoint to pass to spline code
  vector<double> wpts_s, wpts_x, wpts_y, wpts_dx, wpts_dy;
  transform(wpts.begin(), wpts.end(), back_inserter(wpts_s), [](Waypoint const &w) { return w.s; });
  transform(wpts.begin(), wpts.end(), back_inserter(wpts_x), [](Waypoint const &w) { return w.x; });
  transform(wpts.begin(), wpts.end(), back_inserter(wpts_y), [](Waypoint const &w) { return w.y; });
  transform(wpts.begin(), wpts.end(), back_inserter(wpts_dx), [](Waypoint const &w) { return w.dx; });
  transform(wpts.begin(), wpts.end(), back_inserter(wpts_dy), [](Waypoint const &w) { return w.dy; });

  // ensure s element is increasing and doesn't wrap around
  for (int i = 0; i < wpts_s.size(); i++) {
    if (wpts_s[i] < wpts_s[0]) {
      wpts_s[i] += max_s;
    }
  }

  // create splines for each of the Waypoint elements
  tk::spline sp_x, sp_y, sp_dx, sp_dy;
  sp_x.set_points(wpts_s, wpts_x);
  sp_y.set_points(wpts_s, wpts_y);
  sp_dx.set_points(wpts_s, wpts_dx);
  sp_dy.set_points(wpts_s, wpts_dy);

  // calculate number of intermediate points between each map waypoint based on desired distance
  int num_points_inbetween = (wpts_s[wpts_s.size()-1] - wpts_s[0]) / dist;

  // calculate interpolated waypoints based on splines
  // first element is same as original waypoints
  intp_wpts.push_back({.x = wpts[0].x, .y = wpts[0].y, .s = wpts[0].s, .dx = wpts[0].dx, .dy = wpts[0].dy});
  for (int i = 1; i < num_points_inbetween; i++) {
    double s = wpts[0].s + i * dist;          // we deliberately use wpts.s here since it might wrap around
    double x = sp_x(wpts_s[0] + i * dist);    // but for the splines, we account for the wrap around by using wpts_s
    double y = sp_y(wpts_s[0] + i * dist);
    double dx = sp_dx(wpts_s[0] + i * dist);
    double dy = sp_dy(wpts_s[0] + i * dist);
    intp_wpts.push_back({.x = x, .y = y, .s = s, .dx = dx, .dy = dy});
  }

  return intp_wpts;
}
*/

#endif //WAYPOINT
