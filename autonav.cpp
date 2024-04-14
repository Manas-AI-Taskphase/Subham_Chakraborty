#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tfMessage.h>
#include <bits/stdc++.h>
using namespace std;
//GLOBAL DECLARATIONS
float xpose,ypose,zpose;
vector<vector<int>> mapy;
vector<pair<int,int>> turns;
ros::Publisher pub;
double global_angle=-0.0;
double PI = 3.1415926535897;
//functions
void CallBack(struct tf::tfMessage_<std::allocator<void>> x){
   if (x.transforms[0].child_frame_id=="base_footprint"){
      xpose = x.transforms[0].transform.translation.x;
      ypose = x.transforms[0].transform.translation.y;
      zpose = x.transforms[0].transform.translation.z;
      double a = x.transforms[0].transform.rotation.x;
      double y = x.transforms[0].transform.rotation.y;
      double z = x.transforms[0].transform.rotation.z;
      double w = x.transforms[0].transform.rotation.w;
      global_angle= atan2(2*(w*z+a*y),w*w-a*a-y*y-z*z);
   }
   return;
}
void CallBack2(struct nav_msgs::OccupancyGrid_<std::allocator<void>> x){
  int flag =0;
  if (!mapy.empty()){
     flag=1;
  }
  for (int i=0; i<x.info.height;i++){
      vector<int> b(x.info.width);
      for(int j=0; j<x.info.width;j++){
	  if (x.data[i*x.info.width+j]=='d'){
	     b[j] = 1;
	  }
	  else if (x.data[i*x.info.width+j]){
	     b[j] = 7;
	  }
	  else {
	     b[j] = 0;
	  }
      }
      if (flag==1){
        mapy[i]=b;
      }
      else{
        mapy.push_back(b);
      }
    }
  return;
}
int shape(pair<int,int> a, vector<vector<int>> m ,int r){
    // look for 1 in the map inside square. 
    int lowboundy = a.second-r;
    int upboundy = a.second+r;
    int lowboundx = a.first-r;
    int upboundx = a.first+r;
    for (int i=lowboundy; i<=upboundy;i++){
       for(int j=lowboundx; j<=upboundx;j++){
         if (m[i][j] == 1){
             return 0;
         }
       }
    }
    return 1;
}
pair<int,int> transform(float x, float y){
  int xnew = (float)(0.052*384*(x+10));
  int ynew = (float)(0.052*384*(y+10));
  return make_pair(xnew,ynew);
}
int heuristic(pair<int, int> point, pair<int, int> goal) {
    int x = point.first - goal.first;
    int y = point.second - goal.second;
    return abs(x) + abs(y);
}
vector<pair<int,int>> shapystary(vector<vector <int>>& grid,pair<int,int> start,pair<int,int> goal){
    priority_queue<pair<int,pair<int,int>> , vector<pair<int,pair<int,int>>>, greater<pair<int,pair<int,int>>>> f;
    f.push(make_pair(0,start));
    map<pair<int,int>,pair<int,int>> came_from;
    map<pair<int,int>,int> g_score;
    g_score[start] =0;
    map<pair<int,int>,pair<int,int>> direction; //coordinates -> direction, cost
    direction[start].first = 1;
    direction[start].second =0;
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
        vector<int> dir = {0,2,1,4,5,6,8,7};
        for (int i=0; i<8;i++){ //WARNING edited it to 4 instead 8
            int x = current.first + p[i].first;
            int y = current.second + p[i].second;
            pair<int,int> neighbour = make_pair(x,y);
            int dircost = direction[current].second;
            if (direction[current].first != dir[i]){
               dircost =dircost + 10;
            }
            int tentative_g = g_score[current] +10;
            if ((p[i].first==1 || p[i].first==-1) && (p[i].second==1 || p[i].second ==-1)){
                tentative_g+=4;
            }
            if (0<=y && y<grid.size() && 0<=x && x<grid[0].size()){
                // 2 for turtlebot
                if (grid[y][x]==0 or grid[y][x]==7){
                    if (g_score.find(neighbour)==g_score.end()){
                        if(shape(neighbour,grid,4)){
		                g_score[neighbour] = tentative_g;
		                direction[neighbour].first = dir[i];
		                direction[neighbour].second = dircost;
		                int f_score = tentative_g + heuristic(neighbour,goal) + dircost;
		                f.push(make_pair(f_score,neighbour));
		                came_from[neighbour] = current;  
                       }
                      else{
                        grid[y][x] = 5;
                      } 
                    }
                    else{
                        if (tentative_g < g_score[neighbour]){
                            g_score[neighbour] = tentative_g;
                            direction[neighbour].first = dir[i];
                            direction[neighbour].second = dircost;
                            int f_score = tentative_g + heuristic(neighbour,goal)+dircost;
                            f.push(make_pair(f_score,neighbour));
                            came_from[neighbour] = current;
                        }
                    }
                
              }
            }
        }
    }
    cout << "NOT FOUND"<<endl;
    return {};
}
vector<pair<int,int>> Stary(vector<vector <int>>& grid,pair<int,int> start,pair<int,int> goal){
    priority_queue<pair<int,pair<int,int>> , vector<pair<int,pair<int,int>>>, greater<pair<int,pair<int,int>>>> f;
    f.push(make_pair(0,start));
    map<pair<int,int>,pair<int,int>> came_from;
    map<pair<int,int>,int> g_score;
    g_score[start] =0;
    map<pair<int,int>,pair<int,int>> direction; //coordinates -> direction, cost
    direction[start].first = 1;
    direction[start].second =0;
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
        vector<int> dir = {0,2,1,4,5,6,8,7};
        for (int i=0; i<8;i++){ //WARNING edited it to 4 instead 8
            int x = current.first + p[i].first;
            int y = current.second + p[i].second;
            pair<int,int> neighbour = make_pair(x,y);
            int dircost = direction[current].second;
            if (direction[current].first != dir[i]){
               dircost =dircost + 10;
            }
            int tentative_g = g_score[current] +10;
            if ((p[i].first==1 || p[i].first==-1) && (p[i].second==1 || p[i].second ==-1)){
                tentative_g+=4;
            }
            if (0<=y && y<grid.size() && 0<=x && x<grid[0].size()){
                if (grid[y][x]==0 or grid[y][x]==7){
                    if (g_score.find(neighbour)==g_score.end()){
                        g_score[neighbour] = tentative_g;
                        direction[neighbour].first = dir[i];
                        direction[neighbour].second = dircost;
                        int f_score = tentative_g + heuristic(neighbour,goal) + dircost;
                        f.push(make_pair(f_score,neighbour));
                        came_from[neighbour] = current;  
                    }
                    else{
                        if (tentative_g < g_score[neighbour]){
                            g_score[neighbour] = tentative_g;
                            direction[neighbour].first = dir[i];
                            direction[neighbour].second = dircost;
                            int f_score = tentative_g + heuristic(neighbour,goal)+dircost;
                            f.push(make_pair(f_score,neighbour));
                            came_from[neighbour] = current;
                        }
                    }
                
              }
            }
        }
    }
    cout << "NOT FOUND"<<endl;
    return {};
}
vector<int> turny(vector<pair<int,int>>& a){
    int slopex = a[1].first - a[0].first;
    int slopey = a[1].second - a[0].second;
    int c=1;
    vector<int> boly(a.size());
    for(int i=2;i<a.size();i++){
        c++;
        if ((a[i].first-a[i-1].first != slopex) || (a[i].second-a[i-1].second != slopey)){
            slopex = a[i].first - a[i-1].first;
            slopey = a[i].second - a[i-1].second;
            turns.push_back(make_pair(a[i-1].first,a[i-1].second));
            boly[i-1] = 1;
        }
        else if(c%32==0){
            turns.push_back(make_pair(a[i-1].first,a[i-1].second));
        }
    }
    return boly;
}
int close(pair<int,int> a, pair<int,int> b,int n){ //if the point is close to the other point
   for(int i=a.first-n;i<=a.first+n;i++){
      for(int j=a.second-n;j<a.second+n;j++){
          if (i == b.first && j == b.second){
             return 1;
          }
      }
   }
   return 0;
}
pair<int,vector<pair<int,int>>> shapydaddy(vector<pair<int,int>> path){
    vector<pair<int,int>> path2;
    int f =0;
    for(int i=0; i<path.size();i++){
        int x = path[i].first;
        int y = path[i].second;
    	if (!shape(make_pair(x,y),mapy,4)){
    	    f=1;
            pair<int,int> from = make_pair(path[i-2].first, path[i-2].second);
            pair<int,int> to;
            int j=i+1;
            for(; j<path.size();j++){
               i++;
               if (shape(path[j],mapy,4)){
                   to = path[j];
                   break;
               }
               
            }
            if (j==path.size()-1){
               to = path[path.size()-1];
            }
            vector<pair<int,int>> reee = shapystary(mapy,from,to);
            path2.pop_back();
            for(int j=1;j<reee.size();j++){
               path2.push_back(reee[j]);
            }
            continue;
        }
        path2.push_back(make_pair(x,y));
    }
    return make_pair(f,path2);
}
double turn(pair<int,int> x,pair<int,int> y, double k1, int mommy){
  pair<float,float> p1,p2;
  p1.first = x.first/(0.052*384) -10;
  p1.second = x.second/(0.052*384) -10;
  p2.first = y.first/(0.052*384) -10;
  p2.second =  y.second/(0.052*384) -10;
  double k2;
  if (p2.first==p1.first){
      if (p1.second > p2.second){
         k2= -PI/2;
      }
      else{
         k2= PI/2;
      }
  }
  else if (p2.second == p1.second){
     if (p1.first > p2.first){
         k2= PI;
     }
     else{
         k2= 0.0;
     }
  }
  else{
      k2 = atan(((double)(p2.second-p1.second)/(p2.first-p1.first)));
      if (p2.second < p1.second && p2.first < p1.first){
          k2 = k2 - PI;
      }
      if (p2.second > p1.second && p2.first < p1.first){
          k2 = PI + k2;
      }
  }
  cout << "angle2:" << k2 << endl;
  double ang = fabs(k2-k1);
  double speed = 5*(PI/180.0);
  cout << "angle1: "<<k1 << endl;
  cout << "net angle: "<<ang << endl;
  if (ang < 0.01){
     return k2;
  }
  geometry_msgs::Twist vel;
  vel.linear.x=0.0; 
  if (mommy!=0){
     vel.linear.x = 0.00874061438;
  }
  vel.linear.y=0.0;
  vel.linear.z=0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = speed;
  double kk2 = k2;
  if (k2<0){
     kk2 = 2*PI+k2;
  }
  double kk1 = k1;
  if (k1<0){
     kk1 = 2*PI+k1;
  }
  double dif = kk2-kk1;
  if ((dif<0 && (2*PI-fabs(dif)>fabs(dif))) || (dif>0 && (2*PI-fabs(dif)<fabs(dif))))
     vel.angular.z = -speed;
  while(fabs(fabs(global_angle)-fabs(k2))>0.005 || global_angle*k2<0){
     pub.publish(vel);
     ros::spinOnce();
     ros::Duration(0.067).sleep();
  }
  vel.angular.z = 0.0;
  pub.publish(vel);
  return k2;
}
double errorcontroler(){
   double prev = global_angle;
   ros::Duration(0.033).sleep();
   ros::spinOnce();
   double now = global_angle;
   double k = (prev-now)/0.033;
   return k;
}
int main(int argc, char**argv){
    ros::init(argc,argv,"roboy");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/tf",1000,CallBack);
    ros::Subscriber sub2 = n.subscribe("/map",1000,CallBack2);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    while(ros::ok()){
       if (xpose && ypose && !mapy.empty()){
          pair<int,int> start = transform(xpose,ypose);
          cout << start.first << " "<<start.second << endl;
          float xend, yend;
          cout << "Enter end positions: ";
          cin >> xend >> yend;
          pair<int,int> goal = transform(xend,yend);
          cout << goal.first << " "<<goal .second << endl;
          pair<int,int> current = start;
	  vector<pair<int, int>> path = Stary(mapy,start,goal);
	  pair<int,vector<pair<int,int>>>e = shapydaddy(path); //corrector thingy
	  path=e.second;
	  vector<int> boly = turny(path);
	  global_angle = turn(path[0],path[1],global_angle,0);
	  turns.push_back(goal);
	  int i=0;
	  geometry_msgs::Twist vel;
	  vel.linear.y = 0.0;
	  vel.linear.z = 0.0;
	  vel.angular.x = 0.0;
	  vel.angular.y = 0.0;
	  vel.angular.z = 0.0;
	  int kk=0;
	  int z =3;
          while(!close(current,goal,2)){
          	vel.linear.x = 0.05;
          	pub.publish(vel);
          	float curd = 0.0;
          	int county =0;
          	while((!close(current,turns[kk],z)) && kk < turns.size()){
          		ros::spinOnce();
          		vel.angular.z = errorcontroler();
          		if (close(current,goal,2)){
		  	     break;
		  	}
          		pub.publish(vel);
          		current = transform(xpose,ypose);
          		pair<int,vector<pair<int,int>>>e = shapydaddy(path);
          		if (e.first==1){
          		   path = e.second;
          		   turns.clear();
          	           boly = turny(path);
          	           turns.push_back(goal);
          		}
          	        if (county%90==0){
          	           int kkk =0;
          	           for(const auto& point : path){
          	              mapy[point.second][point.first] = 3;
          	              if (point.first == turns[kkk].first && point.second == turns[kkk].second && kkk < turns.size()){
		                  kkk++;
		                  mapy[point.second][point.first] = 4;
		              }
          	           }
          	           for (int l=0; l<mapy.size();l++){
			     for(int j=0; j<mapy[0].size();j++){
				    if (j==current.first && l == current.second){
				       cout << "X" << " ";
				       continue;
				    }
				    if (mapy[l][j] == 4){
				       cout << "T" << " ";
				       continue;
				    }
				    if (mapy[l][j] == 3){
				       cout << '#' << " ";
				       continue;
				    }
				    cout << mapy[l][j]<<" ";
			     }
			     cout << endl;
			  }
          	        }
          	        county++;
          	}
          	ros::spinOnce();
          	ros::Duration(0.067).sleep();
          	if (close(current,goal,2)){
          	     break;
          	}
          	if (kk+1 < turns.size()){
          	   global_angle = turn(current,turns[kk+1],global_angle,1);
          	   kk++;
          	}
          	else if(kk<turns.size()){
          	   global_angle = turn(current,goal, global_angle,1);
          	   kk++;
          	}
          	if (kk>=turns.size()-2){
          	   z=2;
          	}
          	else{
          	   z=3;
          	}
          	ros::spinOnce();
          	pair<int,vector<pair<int,int>>>e = shapydaddy(path);
		if (e.first==1){
		   path = e.second;
		   turns.clear();
		   boly = turny(path);
		   turns.push_back(goal);
		}
          	
          }
          turn(goal,make_pair(goal.first+1,goal.second),global_angle,0); //parking
          vel.linear.x=0.0;
          vel.angular.z=0.0;
          pub.publish(vel);
          break;
       }
       ros::spinOnce();
    }
}
