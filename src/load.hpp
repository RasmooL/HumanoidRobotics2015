// Group C: Rasmus

#include <fstream>
#include <string>
#include <stringstream>
#include <vector>
#include <eigen3/Eigen/Eigen>
using namespace std;

typedef vector<vector<Vector3d>> target_sequence;

void load_msr_skeleton(string fname, target_sequence& left_arm, target_sequence& right_arm)
{
  ifstream file(fname);

  string line;
  getline(file, line);
  stringstream str(line);

  unsigned int frames; // Number of frames in file
  str >> frames;
  unsigned int joints;
  str >> joints; // Should always be 20...

  for(int i = 0; i < frames; i++)
  {
    vector<Vector3d> left_frame, right_frame;
    getline(file, line); // Should always be "40"
    if(line != "40") cerr << "No skeleton/too many skeletons detected." << endl;
    for(int j = 1; j <= 20; j++)
    {
      getline(file, line);
      stringstream s(line);
      Vector3d pos;
      double qx, qy, qz;
      s >> qx >> qy >> qz;
      pos << qz, qx, qy; // This should be the correct order of the axes (I think...)

      switch(j)
      {
        // Right arm
        case 5:
        case 6:
        case 7:
        case 8:
          right_frame.push_back(pos);
          break;

        // Left arm
        case 9:
        case 10:
        case 11:
        case 12:
          left_frame.push_back(pos);
          break;
      }

      getline(file, line); // Skip every 2nd line, since it's useless for us
    }
    left_arm.push_back(left_frame);
    right_arm.push_back(right_frame);
  }

}
