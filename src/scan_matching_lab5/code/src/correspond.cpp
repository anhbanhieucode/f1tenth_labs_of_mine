#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
        for(int j = 0; j<m; ++j){
          float dist = old_points[i].distToPoint2(&trans_points[j]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob)
{

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point. 
  //Initializecorrespondences
  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();

  //Do for each point
  for(int i = 0; i<n; ++i)
  {
    int best = -1;
    double best_dist = numeric_limits<double>::infinity();
    int start_index = i;
    if(start_index<=0){start_index = 0;}
    else if(start_index>=m){start_index=m-1;}
    int we_start_at = (last_best != -1) ? (last_best +1): start_index;
    //search is conducted in two directions: up and down
    int up = we_start_at +1, down = we_start_at;

    //Distance of last point examined in the up (down) direction
    double last_dist_up = 40000.0, last_dist_down = 50000.0;
    //*

    //True if search is finished in the up (down)
    bool up_stopped = false, down_stopped = false;
    while(!(up_stopped && down_stopped))
    {
      bool now_up = !up_stopped & (last_dist_up < last_dist_down);
      if(now_up)
      {
        if(up >= m){up_stopped = true; continue;}
        last_dist_up = pow((trans_points[i].getX()-old_points[up].getX()),2) + pow((trans_points[i].getY()-old_points[up].getY()),2);
        if ( last_dist_up < best_dist)
        {
          best = up;
          best_dist = last_dist_up;
        }

        if(up > start_index)
        {
          double Dphi = old_points[up].theta - trans_points[i].theta;
          double min_dist_up = sin(Dphi)*trans_points[i].r;
          if (pow(min_dist_up,2)>best_dist)
          {
            up_stopped = true;
            continue; //go to the next point in trans_point
          }

          if (old_points[up].r > trans_points[i].r)       
          {
            up = jump_table[up][UP_SMALL];
          }
          else up = jump_table[up][UP_BIG];//jump to bigger
        }

        //when moving toward from start_index
        else
        {
          up++;
        }


      }

       else 
      { //down now
        if( down <= -1)
        {
          down_stopped = true;
          continue;
        }

        

        last_dist_down = pow((trans_points[i].getX() - old_points[down].getX()),2) + pow((trans_points[i].getY() - old_points[down].getY()),2);
        if ( last_dist_down < best_dist)
        {
          best = down;
          best_dist = last_dist_down;
        }
      
        if (down < start_index)
        {
          //Early stopping implement
          //*
          double Dphi = old_points[down].theta - trans_points[i].theta;
          double min_dist_down = sin(Dphi)*trans_points[i].r;
          if (pow(min_dist_down,2)>best_dist)
          {
            down_stopped = true;
            continue; //go to the next point in trans_point
          }
       

        //jumping implement
        //*
        if (old_points[down].r > trans_points[i].r)
        { // jump to smaller
          down = jump_table[down][DOWN_SMALL];
        }
        else down = jump_table[down][DOWN_BIG];//jump to bigger
        }

        //when moving away from start_index
        else
        {
          down--;
        }
        
      }
    
    }




    last_best = best;
    int second_best = 0;

    if (best<=0){
    second_best=best+1;}
    else{
    second_best=best-1;}


      c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
  }
}


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
