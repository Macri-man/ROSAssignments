
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <functional>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>
#include <ctime>

using namespace std;

const int arraywidth=820;
const int arrayheight=700;
const int state = 0;

string tf_prefix;

SDL_Renderer *renderer = NULL;
SDL_Window *win = NULL;
SDL_Texture *texture=NULL;
SDL_Surface *surface=NULL;
SDL_PixelFormat *fmt=NULL;
Uint32 temp, pixel;
Uint8 red, green, blue, alpha;
SDL_Color color;


struct GVertex {
	int x;
	int y;
	GVertex() :x(0), y(0){
	}
	GVertex(int posx, int posy){
		x=posx;
		y=posy;
	}
};

struct GEdges{
	GEdges(){}
	GEdges(GVertex v,GVertex p){
		one.x = v.x;
		one.y = v.y;
		two.x = p.x;
		two.y = p.y;
	}
	GVertex one;
	GVertex two;
};

struct Graph{
	Graph(){}
	Graph(pair<GVertex, GVertex> init, int k,int newdelta){
		maxvertices=k;
		start.x=init.first.x;
		start.y=init.first.y;
		goal.x=init.second.x;
		goal.y=init.second.y;
		vertices.push_back(init.first);
		delta = newdelta;
		root=GNodes(init.second);
		//".c" << create(line, Point(800,800), Point(0,0)) -Tk::fill("black");
	}
	//int delta=20;
	//constint arraywidth=820;
	//constint arrayheight=700;
	//bool cells[820][700];
	int maxvertices;
	int delta;
	GVertex start;
	GVertex goal;
	vector<SDL_Point> renderpoints;
	vector<GEdges> edges;
	vector<GVertex> vertices;
	GNodes root;
}g;

GVertex getRobotPosition();
vector<GVertex> Bresenham(GVertex point1, GVertex point2);
Uint32 getpixel(SDL_Surface *surface, int x, int y);
GVertex normalize(GVertex point);
GVertex nearestVertex(GVertex prand);
bool checkLine(GVertex pnear, GVertex prand);
bool checkPixel(int x, int y);
GVertex new_conf(GVertex pnear,GVertex prand,int delta);
pair<GVertex, GVertex> rand_conf();
Graph RRT(pair<GVertex,GVertex> qinit,int kvertices,int delta);
void add_vertex(GVertex pnear,GVertex pnew);
float dist(GVertex a,GVertex b);
void getInput();


/*PSEUDO

Algorithm BuildRRT
  Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq)
  Output: RRT graph G

  G.init(qinit)
  for k = 1 to K
    qrand ← RAND_CONF()
    qnear ← NEAREST_VERTEX(qrand, G)
    qnew ← NEW_CONF(qnear, qrand, Δq)
    G.add_vertex(qnew)
    G.add_edge(qnear, qnew)
  return G


*/

  /* refrence def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)
*/

float dist(GVertex a,GVertex b){
	return sqrt(((a.x-b.x)*(a.x-b.x))+((a.y-b.y)*(a.y-b.y)));
}


GVertex normalize(GVertex point){
	return GVertex((point.x/(sqrt((point.x*point.x)+(point.y*point.y)))),(point.y/(sqrt((point.x*point.x)+(point.y*point.y)))));
}

Uint32 getpixel(SDL_Surface *surface, int x, int y){
    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to retrieve */
    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        return *p;
        break;

    case 2:
        return *(Uint16 *)p;
        break;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
            return p[0] << 16 | p[1] << 8 | p[2];
        else
            return p[0] | p[1] << 8 | p[2] << 16;
        break;

    case 4:
        return *(Uint32 *)p;
        break;

    default:
        return 0;       /* shouldn't happen, but avoids warnings */
    }
}

GVertex nearestVertex(GVertex prand){
	cerr << "Nearest Vertex" << endl;
	GVertex temp=GVertex(g.vertices[0]);
	for(int i=0;i<g.vertices.size();++i){
		if(dist(g.vertices[i],prand)<dist(temp,prand)){
			temp.x=g.vertices[i].x;
			temp.y=g.vertices[i].y;
		}
	}
	return temp;
}

//returns true if white
bool checkPixel(int x, int y){
	SDL_LockSurface(surface);
	int pixel = getpixel(surface,x,y);
	SDL_UnlockSurface(surface);
	SDL_GetRGBA(pixel,surface->format,&red,&green,&blue,&alpha);
	fprintf(stderr, "Pixel info x: %d y: %d -> R: %d  G: %d B: %d\n",x,y,red,green,blue);
	return (red==255 && green==255 && blue==255);
}


//true if line is white
bool checkLine(GVertex pnear, GVertex prand){
	cerr << "CheckLine" << endl;

	vector<GVertex> v(Bresenham(pnear,prand));
		for(int i=0;i<v.size();++i){
			if(!checkPixel(v[i].x,v[i].y)){
				return false;
			}
		}
		return true;
}

bool isGoodtoStart(GVertex pnear,GVertex prand){
	return checkLine(pnear,prand);
}

/* 	check for vertical line
	check for horizontal line 
	
	check if start is greater than end
	if false
		switch
	get points on line	

*/


vector<GVertex> Bresenham(GVertex start, GVertex end){
cerr << "Bresenham line:" << endl;
	vector<GVertex> v;

	if(start.x==end.x){

		if(start.y<end.y){

			v.push_back(start);

			for(int y=start.y+1;y<=end.y;++y){
				v.push_back(GVertex(start.x,y));
			}

		}else{

			v.push_back(end);

			for(int y=end.y+1;y<=start.y;++y){
				v.push_back(GVertex(end.x,y));
			}

		}
	}else if(start.y==end.y){

		if(start.x<end.x){

			v.push_back(start);

			for(int x=start.x+1;x<=end.x;++x){
				v.push_back(GVertex(x,start.y));
			}

		}else{

			v.push_back(end);

			for(int y=end.y+1;y<=start.y;++y){
				v.push_back(GVertex(end.x,y));
			}

		}
	}else{

		if((start.x<end.x)&&(start.y<end.y)){
			v.push_back(start);

			int dx = end.x-start.x;
  			int dy = end.y-start.y;

  			int err = 2*dy - dx;
  			int y = start.y;

  			for(int x = start.x+1;x <= end.x; ++x){
    			err = err + (2*dy);
    			if(err > 0){
      				y = y+1;
      				err = err - (2*dx);
      				v.push_back(GVertex(x,y));
  				}
  			}
  		}else{
  			v.push_back(end);

			int dx = start.x-end.x;
  			int dy = start.y-end.y;

  			int err = 2*dy - dx;
  			int y = end.y;

  			for(int x = end.x+1;x <= start.x; ++x){
    			err = err + (2*dy);
    			if(err > 0){
      				y = y+1;
      				err = err - (2*dx);
      				v.push_back(GVertex(x,y));
  				}
  			}
  		}
	}


	for(int i=0;i<v.size();++i){
		cerr << "Point on line: "<< i << " x: " << v[i].x << " y: " << v[i].y <<  endl;
	}

  return v;
}

GVertex new_conf(GVertex pnear,GVertex prand,int delta){
	cerr << "NEW Configuration" << endl;
	if(dist(pnear,g.start) < delta){
		cerr << "GO TO START" << endl;
		return g.start;
	}else if(dist(pnear,prand) < delta){
		cerr << "Dist: " << dist(pnear,prand) << "Less Than: " << delta << endl;
		return prand;
	}else{
		return GVertex(pnear.x+delta,pnear.y+delta);
	}
}

pair<GVertex, GVertex> rand_conf(){
	cerr << "Random Configuration" << endl;

	int x,y;
	GVertex p;

	//int maxx=robpos.x+g.delta,minx=robpos.x-delta;

	robpos=getRobotPosition();

	while(true){
		//rand()%(max-min + 1) + min;
		x = rand() % (arraywidth) + 1;
		y = rand() % (arrayheight) + 1;
		SDL_LockSurface(surface);
		pixel = getpixel(surface,x,y);
		SDL_UnlockSurface(surface);
		SDL_GetRGBA(pixel,surface->format,&red,&green,&blue,&alpha);
		p = GVertex(nearestVertex(GVertex(x,y)));
		if(checkLine(p,GVertex(x,y))){
			break;
		}

		//cerr << "Pixel Color -> R: "<< (int)red << " G: " << (int)green << " B: " << (int)blue <<  " A: " << (int)alpha << endl;
		fprintf(stderr, "Pixel %d x: %d y: %d Color -> R: %d G: %d B: %d A: %d \n",pixel,x,y,red,green,blue,alpha);
	}

	//return nearestVertex(GVertex(x,y));
	return make_pair(p,GVertex(x,y));
}


void add_vertex(GVertex pnear,GVertex pnew){
	cerr << "Add Vertex" << endl;
	SDL_Point p={pnew.x,pnew.y};
	//SDL_Point p={pnear.x,pnear.y};
	g.renderpoints.push_back(p);
	g.vertices.push_back(pnew);
	g.edges.push_back(GEdges(pnear,pnew));
}

Graph RRT(pair<GVertex,GVertex> qinit,int kvertices,int delta){
	GVertex prand,pnear,pnew;
	g=Graph(qinit,kvertices,delta);
	for(int i = 1; i <= g.maxvertices; ++i){
		cerr << "Iteration: " << i << endl;
		pair<GVertex, GVertex> p=rand_conf();
		//pnear=nearestVertex(prand);
		pnew=new_conf(p.first,p.second,g.delta);
		add_vertex(p.first,pnew);


		//SDL_RenderPresent(renderer);
		//cerr << "List of points: " << i << endl;
		cerr << "NEW point: " << i << endl;
		cerr << "X: " << g.vertices[g.vertices.size()-1].x << " Y: " << g.vertices[g.vertices.size()-1].y << endl;
		//for(int k=0;k<g.renderpoints.size();k++){
			//cerr << "X: " << g.renderpoints[k].x << " Y: " << g.renderpoints[k].y << endl;
		//}

		SDL_RenderClear(renderer);
    	SDL_RenderCopy(renderer, texture, NULL, NULL);
		/*if(SDL_RenderDrawLines(renderer,&g.renderpoints[0],g.renderpoints.size())!=0){
        	fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        	exit(1);
    	}*/

        for(int e=0;e<g.edges.size();++e){
       		if(SDL_RenderDrawLine(renderer,g.edges[e].one.x,g.edges[e].one.y,g.edges[e].two.x,g.edges[e].two.y)!=0){
        		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        		exit(1);
    		}
    	}

    	//SDL_RenderSetScale( renderer, 3, 3 );

    	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 0);
    	SDL_RenderDrawPoint(renderer,g.goal.x,g.goal.y);
    	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);
    	SDL_RenderDrawPoint(renderer,g.start.x,g.start.y);
    	SDL_SetRenderDrawColor(renderer, 0, 0, 255, 0);
    	//SDL_RenderSetScale( renderer, 1, 1 );

    	SDL_RenderPresent(renderer);
    	//SDL_RenderPresent(renderer);

    	if((pnew.x==g.start.x)&&(pnew.y==g.start.y)){
    		cerr << "Graph complete" << endl;
    		while(true){
    			getInput();
    		}
    		//SDL_RenderPresent(renderer);
    	}

    	getInput();
    	SDL_Delay(500);

	}
	return g;
}

void getInput(){
	SDL_Event e;
        while(SDL_PollEvent(&e)) {
        	switch(e.type){
        		case SDL_QUIT:
        			exit(1);
        			break;
        		case SDL_KEYDOWN:
        		case SDL_KEYUP:
        			switch(e.key.keysym.sym){
        				case SDLK_BACKSPACE:
        				case SDLK_ESCAPE:
        					exit(1);
        			}
        			break;
        		default:
        			continue;
        	}
            
        }
}

GVertex getRobotPosition(){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    GVertex pos;

    try {
        string base_footprint_frame = tf::resolve(tf_prefix, "base_footprint");
	
        listener.waitForTransform("/odom", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", base_footprint_frame, ros::Time(0), transform);

        GVertex(transform.getOrigin().x(),transform.getOrigin().y());
    }
    catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
    }
    return pos;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "stage");
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	nh_.getParam("tf_prefix", tf_prefix);
	geometry_msgs::Twist base_cmd;
	string input;
	int state;

	int x,y;
	srand (time(NULL));

	if(SDL_Init(SDL_INIT_EVERYTHING)!=0){
        fprintf(stderr, "Error: Unable to init SDL: %s\n", SDL_GetError());
        exit(1);
    }

    int image_flags = IMG_INIT_PNG;
	if(IMG_Init(image_flags) != image_flags){
    	cout << "Error: " << IMG_GetError() << endl;
	}

    win = SDL_CreateWindow("RRT GRAPH", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,arraywidth, arrayheight,SDL_WINDOW_SHOWN);

    if(!win){
       	fprintf(stderr, "Unable to initilize SDL2: %s\n", SDL_GetError());
       	SDL_Quit();
       	exit(1);
    }

    string filepath = "/home/csguest/catkin_ros/src/Stage/bitmaps/autolab.png";

    surface=IMG_Load(filepath.c_str());
    //surface = SDL_LoadBMP(../Stage/bitmaps/autolab.png);
    //surface = CreateRGBSurfaceFrom(,width,height,32,0,0,0,0);

    if(!surface){
    	fprintf(stderr, "Create Surface failed: %s\n", SDL_GetError());
        exit(1);
    }



    //SDL_LockSurface(surface);

    /*
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, 1024, 768);

    if(!texture){
    	fprintf(stderr, "Unable to create texture SDL2: %s\n", SDL_GetError());
       	SDL_Quit();
       	exit(1);
    }*/

    renderer = SDL_CreateRenderer(win, -1, 0);
    texture=SDL_CreateTextureFromSurface(renderer, surface);

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);

   	//SDL_RenderFillRect( renderer, &r );
    //Render the rect to the screen
    //SDL_RenderPresent(renderer);
    //SDL_SetRenderDrawColor(SDL_Renderer* renderer,0,0,0,255)

	cout << "What is the intial location on a map x,y" << endl;
	do{
		cerr << "Make sure your position is not inside a obsticle" << endl;
		cin >> x >> y;
	}while(!checkPixel(x,y));

	GVertex start=GVertex(x,y);

	cout << "What is the goal location on a map x,y" << endl;
	do{
		cerr << "Make sure your position is not inside a obsticle" << endl;
		cin >> x >> y;
	}while(!checkPixel(x,y));

	GVertex goal=GVertex(x,y);

	ros::Rate loop_rate(10);
	pair<GVertex,GVertex> ginit = make_pair(start,goal);

	RRT(ginit,3000,10);

	while(ros::ok()){
		
		cmd_vel_pub_.publish(base_cmd);
		

		ros::spinOnce();
		loop_rate.sleep();
	}

	SDL_DestroyRenderer(renderer);
	SDL_DestroyTexture(texture);
	SDL_FreeSurface(surface);
	SDL_DestroyWindow(win);
    SDL_Quit();

	return 0;
}
