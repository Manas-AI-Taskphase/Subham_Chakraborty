#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <bits/stdc++.h>
using namespace std;
ros::Publisher pub;
int heuristic(pair<int, int> point, pair<int, int> goal) {
    int x = point.first - goal.first;
    int y = point.second - goal.second;
    return (int)(sqrt(x*x+y*y)*10);
}
vector<pair<int,int>> Stary(vector<vector <int>>& grid){
  for (int i=0; i<grid.size();i++){
      for(int j=0; j<grid[0].size();j++){
         cout << grid[i][j]<<" ";
      }
      cout << endl;
  }
  pair <int,int> start = make_pair(0,0);
  pair <int,int> goal = make_pair(grid.size()-1,grid[0].size()-1);
  priority_queue<pair<int,pair<int,int>> , vector<pair<int,pair<int,int>>>, greater<pair<int,pair<int,int>>>> f;
    f.push(make_pair(0,start));
    map<pair<int,int>,pair<int,int>> came_from;
    map<pair<int,int>,int> g_score;
    g_score[start] =0;
    while (!f.empty()){
        pair<int,int> current = f.top().second;
        f.pop();
        if (current.first==goal.first && current.second==goal.second){
            vector<pair<int,int>> path;
            while (came_from.find(current)!=came_from.end()){
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start);
            reverse(path.begin(),path.end());
            return path;
        }
        vector<pair<int,int>> p = {make_pair(0,1),make_pair(0,-1),make_pair(1,0),make_pair(-1,0),make_pair(1,1),make_pair(1,-1),make_pair(-1,1),make_pair(-1,-1)};
        for (int i=0; i<8;i++){
            int x = current.first + p[i].first;
            int y = current.second + p[i].second;
            pair<int,int> neighbour = make_pair(x,y);
            int tentative_g = g_score[current] +10;
            if ((p[i].first==1 || p[i].first==-1) && (p[i].second==1 || p[i].second ==-1)){
                tentative_g+=4;
            }
            if (0<=x && x<grid.size() && 0<=y && y<grid[0].size()){
                if (grid[x][y]==0){
                    if (g_score.find(neighbour)==g_score.end()){
                        g_score[neighbour] = tentative_g;
                        int f_score = tentative_g + heuristic(neighbour,goal);
                        f.push(make_pair(f_score,neighbour));
                        came_from[neighbour] = current;
                    }
                    if (g_score.find(neighbour)!=g_score.end()){
                        if (tentative_g < g_score[neighbour]){
                            g_score[neighbour] = tentative_g;
                            int f_score = tentative_g + heuristic(neighbour,goal);
                            f.push(make_pair(f_score,neighbour));
                            came_from[neighbour] = current;
                        }
                    }
                }
            }
        }
    }
    return {};
}
void publisher(vector<pair<int,int>>& pathy,int m,int n){
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    for (int i=0;i<pathy.size();i++){
       pose.pose.position.x = pathy[i].second;
       pose.pose.position.y = pathy[i].first;
       pose.pose.position.z = 0;
       path.poses.push_back(pose);
    }
    pub.publish(path);
}
void CallBack(struct nav_msgs::OccupancyGrid_<std::allocator<void>> x){
  vector<vector<int>> a(x.info.height);
  for (int i=0; i<x.info.height;i++){
      vector<int> b(x.info.width);
      for(int j=0; j<x.info.width;j++){
	  if (x.data[i*x.info.width+j]){
	     b[j] = 1;
	  }
	  else {
	     b[j] = 0;
	  }  
      }
      a[i] = b;
  }
  vector<pair<int, int>> path = Stary(a);
    for (const auto& point : path) {
        a[point.first][point.second] = 2;
        cout << "(" << point.first << ", " << point.second << ") ";
    }
    cout << endl;
    for (int i=0; i<a.size();i++){
        for(int j=0; j<a[0].size();j++){
            if (a[i][j] == 2){
               cout << 'X' << " ";
               continue;
            }
            cout << a[i][j]<<" ";
        }
        cout << endl;
    }
    std::cout << std::endl;
    publisher(path,x.info.height,x.info.width);
 }
int main(int argc, char**argv){
    ros::init(argc,argv,"pather");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/map",1000,CallBack);
    pub = n.advertise<nav_msgs::Path>("/path",1000);
    ros::spin();
}
