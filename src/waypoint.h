//
// Helper functions for Path Planning demo
//

#ifndef WAYPOINT
#define WAYPOINT

#include <math.h>
#include <vector>

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
int ClosestWaypoint(double x, double y, const vector<Waypoint> &maps)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps.size(); i++)
  {
    double map_x = maps[i].x;
    double map_y = maps[i].y;
    double dist = xyDistance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<Waypoint> &maps)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps);

  double map_x = maps[closestWaypoint].x;
  double map_y = maps[closestWaypoint].y;

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<Waypoint> &maps)
{
  int next_wp = NextWaypoint(x,y, theta, maps);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps.size()-1;
  }

  double n_x = maps[next_wp].x-maps[prev_wp].x;
  double n_y = maps[next_wp].y-maps[prev_wp].y;
  double x_x = x - maps[prev_wp].x;
  double x_y = y - maps[prev_wp].y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = xyDistance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps[prev_wp].x;
  double center_y = 2000-maps[prev_wp].y;
  double centerToPos = xyDistance(center_x,center_y,x_x,x_y);
  double centerToRef = xyDistance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += xyDistance(maps[i].x,maps[i].y,maps[i+1].x,maps[i+1].y);
  }

  frenet_s += xyDistance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<Waypoint> &maps)
{
  int prev_wp = -1;

  while(s > maps[prev_wp+1].s && (prev_wp < (int)(maps.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps.size();

  double heading = atan2((maps[wp2].y-maps[prev_wp].y),(maps[wp2].x-maps[prev_wp].x));
  // the x,y,s along the segment
  double seg_s = (s-maps[prev_wp].s);

  double seg_x = maps[prev_wp].x+seg_s*cos(heading);
  double seg_y = maps[prev_wp].y+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

#endif //WAYPOINT
