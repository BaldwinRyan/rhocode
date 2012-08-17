#include <iostream>
#include <math.h>

using namespace std;

int main() {
  const float PI = 3.14159;

  float theta = 0; //Current Orientation measured from North (radians)
  float x = 0;    //Current X Position (m)
  float y = 0;   //Current Y Position (m)
  //These arrays give the X and Y coordinates of the destinations (m)
  float xDest[] = {0, 4, 2, 8, 5,-2};
  float yDest[] = {0,-2, 0, 4, 4,10};
  //float xDest[] = {50, 70};
  //float yDest[] = {-30,-46};

  float v = .2;        //Constant forward speed (m/s)
  float w;            //Angular Speed (rad/s) [not constant]
  float k = .7;      //Constant used to compute angular speed
  float d;          //Distance from current destination (m)
  //In this range of values the angular speed depends on the distance from the destination
  float dmin = 4;      //Min Distance (m)
  float dmax = 9;     //Max Distance (m)
  float heading;     //Direction from current position to destination measured from North (radians)
  float deltaTheta; //Angle between current orientation and heading (radians)

  float dt = 1;   //time to step each iteration (s)

  //Iterate for every destination
  for (int j=0; j<6; j++) {
    //compute the distance to the destination
    d = sqrt((xDest[j]-x)*(xDest[j]-x) + (yDest[j]-y)*(yDest[j]-y));
    
    //Go until the robot is within 1 meter of the destination
    while (d > 1) {
      d = sqrt((xDest[j]-x)*(xDest[j]-x) + (yDest[j]-y)*(yDest[j]-y));
      cout << x << " " << y << " "<<d <<"\n";
      
      //Compute the heading
      if (xDest[j] > x)
        heading = -PI/2 + atan((yDest[j]-y)/(xDest[j]-x));
      else if (xDest[j] < x)
        heading =  PI/2 + atan((yDest[j]-y)/(xDest[j]-x));
      else if (yDest[j] > y)
        heading = 0;
      else
        heading = -PI;


      //Compute Delta Theta
      deltaTheta = heading - theta;
      if (deltaTheta >  PI) deltaTheta -= 2*PI;
      if (deltaTheta < -PI) deltaTheta += 2*PI;
      
      //Compute the angular speed
      float w0 = w;
      if (d < dmin) {
        w = k * deltaTheta / dmin;
      } else if (d > dmax) {
        w = k * deltaTheta / dmax;
      } else {
        w = k * deltaTheta / d;
      }
      //This part ensures that the angular acceleration is not too large
      if ((w-w0) >  .1) w = w0 + .1;
      if ((w-w0) < -.1) w = w0 - .1;

      //Update position and orientation
      x -= v*dt*sin(theta);
      y += v*dt*cos(theta);
      theta += w*dt;
    }
  }
  return 0;
}
