#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>


class Utilities
{
  public:
    Utilities();
    Utilities(int);
    ~Utilities();

    double obtainCurvature(const double& x, const double& y);
    double obtainCurvatureFromS(const double& s);
    int identifyRegion(const double& x, const double& y);
    int identifyRegionFromS(const double& s);
    // void loadMapParameters(int);
    
    std::vector<std::vector<double>> getMapParameters();
  private:
    
    double map_parameters_[2][5][6] = 
    {
      
      {
        {-8.62, 8.38, 2.36, 0, 5.9821, 0},           
        {-16.94, -0.19, -0.80, 5.9821, 18.7621, 0.1674}, 
        {-8.81, -8.64, -0.80, 24.7552, 11.726, 0}, 
        {-0.12, 0.05, 2.36, 36.4702, 19.304, 0.1627}, 
        {-4.37, 4.17, 2.36, 55.774, 5.919, 0}
      },

      {
        {2.78, -2.97, -0.7333,0, 3.8022, 0},
        {10.04, 6.19, 2.2844, 3.8022, 18.3537, 0.1712}, 
        {1.46, 13.11, 2.2844, 22.1559, 11.0228, 0},
        {-5.92, 3.80, -0.7333, 33.1787, 18.666, 0.1683},
  {-0.24, -0.66, -0.7333, 51.8453, 7.2218, 0}
      }
    };
    // std::string map_parameters_file_[4] = {"Map_Parameters/MariettaGazebo.csv", "Map_Parameters/MariettaTrack.csv", "Map_Parameters/CCRFGazebo.csv", "Map_Parameters/CCRFTrack.csv"};
    int map_flag_;
};

Utilities::Utilities(){
  Utilities(0);
}

Utilities::Utilities(int map_flag)
{
  map_flag_ = map_flag;
  // loadMapParameters(map_flag_);
}

Utilities::~Utilities()
{}

// std::vector<std::vector<double>> Utilities::getMapParameters()
// {
//   return (map_parameters_);
// }

// void Utilities::loadMapParameters(int map_flag)
// {
//   // Load Map Parameters from a file
//     map_flag_ = map_flag;
//     std::vector<double> parameter_i(6, 0.0);
//     char comma;
//     std::ifstream parametersFile;
//     parametersFile.open(map_parameters_file_[map_flag_], std::ifstream::in | std::ifstream::binary);

//     if (parametersFile.is_open())
//     {
//       std::cout << "File is open" << std::endl;
//       while (!parametersFile.eof())
//       {

//         for(int i = 0; i < 6; i++)
//         {
    
//           parametersFile >> parameter_i[i];
//           parametersFile >> comma;
//         }

//         map_parameters_.push_back(parameter_i);
//       }
//     }
//     parametersFile.close();

// }

double Utilities::obtainCurvature(const double& x, const double& y)
{
  int current_region = identifyRegion(x, y);
  return (map_parameters_[map_flag_][current_region][5]);
}

double Utilities::obtainCurvatureFromS(const double& s)
{
  int current_region = identifyRegionFromS(s);
  return (map_parameters_[map_flag_][current_region][5]);
}
    
int Utilities::identifyRegion(const double& x, const double& y)
  {
    // Identify the region by dividing the map into a set of straight lines to denote different regions and finding where the point lies wrt these lines 
    if (map_flag_ == 0) 
    {
      if ( y >= (map_parameters_[map_flag_][0][1] - map_parameters_[map_flag_][1][1]) / (map_parameters_[map_flag_][0][0] - map_parameters_[map_flag_][1][0]) * (x - map_parameters_[map_flag_][0][0]) + map_parameters_[map_flag_][0][1])
        return (1);
      else if ( y <= (map_parameters_[map_flag_][3][1] - map_parameters_[map_flag_][2][1]) / (map_parameters_[map_flag_][3][0] - map_parameters_[map_flag_][2][0]) * (x - map_parameters_[map_flag_][2][0]) + map_parameters_[map_flag_][2][1])
        return (3);
      else
      {
        double x01 = 0.5 * (map_parameters_[map_flag_][0][0] + map_parameters_[map_flag_][1][0]);
        double y01 = 0.5 * (map_parameters_[map_flag_][0][1] + map_parameters_[map_flag_][1][1]);
        double x23 = 0.5 * (map_parameters_[map_flag_][2][0] + map_parameters_[map_flag_][3][0]);
        double y23 = 0.5 * (map_parameters_[map_flag_][2][1] + map_parameters_[map_flag_][3][1]);
        double inc = (y01 - y23)/(x01 - x23);

        if (y <= inc * (x-x01) + y01)
                return (2);
            else if ( y >= - 1.0 /inc * (x - map_parameters_[map_flag_][4][0]) + map_parameters_[map_flag_][4][1] )
                return (0);
            else
              return (4);
          
      }
    }

    else if (map_flag_ == 1)
    {
      if ( y <= (map_parameters_[map_flag_][0][1] - map_parameters_[map_flag_][1][1]) / (map_parameters_[map_flag_][0][0] - map_parameters_[map_flag_][1][0]) * (x - map_parameters_[map_flag_][0][0]) + map_parameters_[map_flag_][0][1])
        return (1);
      else if ( y >= (map_parameters_[map_flag_][3][1] - map_parameters_[map_flag_][2][1]) / (map_parameters_[map_flag_][3][0] - map_parameters_[map_flag_][2][0]) * (x - map_parameters_[map_flag_][2][0]) + map_parameters_[map_flag_][2][1])
        return (3);
      else
      {
        double x01 = 0.5 * (map_parameters_[map_flag_][0][0] + map_parameters_[map_flag_][1][0]);
        double y01 = 0.5 * (map_parameters_[map_flag_][0][1] + map_parameters_[map_flag_][1][1]);
        double x23 = 0.5 * (map_parameters_[map_flag_][2][0] + map_parameters_[map_flag_][3][0]);
        double y23 = 0.5 * (map_parameters_[map_flag_][2][1] + map_parameters_[map_flag_][3][1]);
        double inc = (y01 - y23)/(x01 - x23);

        if (y >= inc * (x-x01) + y01)
                return (2);
            else if ( y <= - 1.0 /inc * (x - map_parameters_[map_flag_][4][0]) + map_parameters_[map_flag_][4][1] )
                return (0);
            else
              return (4);
          
      } 
    }
    
  }
int Utilities::identifyRegionFromS(const double& s)
  {
    for(int i = 0; i < 4; ++i)
    {
      if(map_parameters_[map_flag_][i][3] <=s && s < map_parameters_[map_flag_][i+1][3]){
        return i;
      }
    }
    return (4);
  }
#endif 
