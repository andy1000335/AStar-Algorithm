#include "ImageReader.h"
#include <stdio.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <stack>
#include <queue>

using namespace std;
using namespace cv;

struct point {
	int x = -1;
	int y = -1;
	int length=0;
	int distance=0;
};
struct MouseParams {
	Mat image;
	point* start;
	point* goal;
};

uchar* Mat2Array (Mat input) {
	int width = input.cols;
	int height = input.rows;
	int channel = input.channels();
	uchar* imgArray = (uchar*)malloc(width*height*channel*sizeof(uchar));
	if (channel == 1) {
		for (int row=0; row<height; row++)
			for (int col=0; col<width; col++)
				imgArray[row*width + col] = input.at<uchar>(row, col);
	}
	else {
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				for (int c = 0; c < channel; c++) {
					imgArray[(row*width + col)*channel + c] = input.at<Vec3b>(row, col)[c];
				}
			}
		}
	}
	return imgArray;
}

uchar* padding (uchar* input, int width, int height, int num) {
	uchar* output = (uchar*)calloc((width+num*2)*(height+num*2), sizeof(uchar));
	for (int row=0; row<height; row++)
		for (int col=0; col<width; col++)
			output[(row+num)*(width+num*2) + (col+num)] = input[row*width + col];
	for (int n=0; n<num; n++) {
		for (int col=num; col<(width+num); col++) {
			output[(num-1-n)*(width+num*2) + col] = output[(num+n)*(width+num*2) + col];
			output[(height+num+n)*(width+num*2) + col] = output[(height+num-1-n)*(width+num*2) + col];
		}
		for (int row=num; row<(height+num); row++) {
			output[row*(width+num*2) + (num-1-n)] = output[row*(width+num*2) + (num+n)];
			output[row*(width+num*2) + (width+num+n)] = output[row*(width+num*2) + (width+num-1-n)];
		}
	}
	for (int row=0; row<num; row++) {
		for (int col=0; col<num; col++){
			output[row*(width+num*2) + col] = output[(2*num-1-col)*(width+num*2) + (2*num-1-row)];
			output[row*(width+num*2) + (col+width+num)] = output[(col+num)*(width+num*2) + (row+width)];
			output[(row+height+num)*(width+num*2) + col] = output[(height+num-num+col)*(width+num*2) + (row+num)];
			output[(row+height+num)*(width+num*2) + (col+width+num)] = output[(height+num-1-col)*(width+num*2) + (width+num-1-row)];
		}
	}

	return output;
}

uchar* thinning (uchar* input, int width, int height) {
	/*
		[ 0 0 0    [ x 0 0    [ 1 x 0    [ 1 1 x    [ 1 1 1    [ x 1 1    [ 0 x 1    [ 0 0 x
		  x 1 x      1 1 0      1 1 0      1 1 0      x 1 x      0 1 1      0 1 1      0 1 1
		  1 1 1 ]    1 1 x ]    1 x 0 ]    x 0 0 ]    0 0 0 ]    0 0 x ]    0 x 1 ]    x 1 1 ]
	*/
	int kNum = 8;
	int kSize = 3;
	int kernel[] = { 0,  0,  0, -1,  1, -1,  1,  1,  1,     // B1
					-1,  0,  0,  1,  1,  0,  1,  1, -1,     // B2
					 1, -1,  0,  1,  1,  0,  1, -1,  0,     // B3
					 1,  1, -1,  1,  1,  0, -1,  0,  0,     // B4
					 1,  1,  1, -1,  1, -1,  0,  0,  0,     // B5
					-1,  1,  1,  0,  1,  1,  0,  0, -1,     // B6
					 0, -1,  1,  0,  1,  1,  0, -1,  1,     // B7
					 0,  0, -1,  0,  1,  1, -1,  1,  1};    // B8
	bool remove;
	uchar* paddingImg = padding(input, width, height, 1);
	uchar* temp = (uchar*)malloc((width+2)*(height+2)*sizeof(uchar));
	uchar* output = (uchar*)malloc(width*height*sizeof(uchar));
	memcpy(temp, paddingImg, (width+2)*(height+2));
	memcpy(output, input, width*height);
	int kValue;
	for (int k=0; k<kNum; k++) {
		memcpy(paddingImg, temp, (width+2)*(height+2));
		for (int row=1; row<height+1; row++) {
			for (int col=1; col<width+1; col++) {
				remove = true;
				for (int r=-1; r<kSize-1; r++) {
					for (int c=-1; c<kSize-1; c++) {
						kValue = kernel[k*kSize*kSize + (r+1)*kSize + (c+1)];
						if (kValue != -1) {
							remove = ((!paddingImg[(row+r)*(width+2)+(col+c)] && !kValue)  ||
									  ( paddingImg[(row+r)*(width+2)+(col+c)] &&  kValue)) && remove;
						}
					}
				}
				if (remove) {
					temp[row*(width+2) + col] = 0;
					output[(row-1)*width + (col-1)] = 0;
				}
			}
		}
	}
	free(paddingImg);
	free(temp);
	return output;
}

void A_star (uchar* map, int width, int height, point start, point goal, queue<stack<point>> allRoads={}) {
	printf("\rSearching for shortest path . . .");
	if (map[start.y*width+start.x]!=255 || map[goal.y*width+goal.x]!=255) {
		cout << endl << "point error" << endl;
		memcpy(map, (uchar*)calloc(width*height, sizeof(uchar)), width*height);
		return;
	}

	if (allRoads.empty()) {
		int distance = round(sqrt(pow(start.x-goal.x, 2)+pow(start.y-goal.y, 2)));
		stack<point> initRoad;
		initRoad.push({start.x, start.y, 0, distance});
		allRoads.push(initRoad);
	}

	// compare weight
	int size = allRoads.size();
	int idx = 0;
	int minCost = sqrt(pow(start.x-goal.x, 2)+pow(start.y-goal.y, 2)) + width*height;
	int cost;
	while (size-- > 0) {
		stack<point> road = allRoads.front();
		allRoads.pop();
		allRoads.push(road);
		point p = road.top();
		cost = p.length + p.distance;
		if (cost < minCost) {
			minCost = cost;
			idx = allRoads.size() - size - 1;
		}
	}

	// get least cost road
	while (idx-- > 0) {
		stack<point> temp = allRoads.front();
		allRoads.pop();
		allRoads.push(temp);
	}
	stack<point> leastCostRoad = allRoads.front();
	point now = leastCostRoad.top();

	// recursive
	if (now.x==goal.x && now.y==goal.y) {
		for (int index=0; index<width*height; index++)
			map[index] = 0;
		stack<point> road = allRoads.front();
		while (!road.empty()) {
			point p = road.top();
			map[p.y*width + p.x] = 255;
			road.pop();
		}
		cout << endl << "Search complete." << endl;
	}
	else {
		point last;
		if (leastCostRoad.size() == 1)
			last = now;
		else {
			leastCostRoad.pop();
			last = leastCostRoad.top();
			leastCostRoad.push(now);
		}
		bool hasCross = false;
		bool death = true;
		int nextX, nextY;
		stack<point> road;
		for (int x=-1; x<2; x++) {
			for (int y=-1; y<2; y++) {
				nextX = now.x+x;
				nextY = now.y+y;
				if (nextX<0 || nextX>width-1 || nextY<0 || nextY>height-1    // boundry
					|| (nextX==last.x && nextY==last.y) 
					|| (map[nextY*width + nextX]!=255)
					|| x==y || x+y==0)                                       // D4
					continue;
				if (nextX!=goal.x && nextY!=goal.y)
					map[nextY*width + nextX] = 0;
				death = false;
				int distance = round(sqrt(pow(nextX-goal.x, 2)+pow(nextY-goal.y, 2)));
				if (hasCross) {
					road = leastCostRoad;
					road.push(point{nextX, nextY, int(leastCostRoad.size()), distance});
					allRoads.push(road);
				} else {
					allRoads.pop();
					road = leastCostRoad;
					road.push(point{nextX, nextY, int(leastCostRoad.size()), distance});
					allRoads.push(road);
					hasCross = true;
				}
			}
		}
		if (death) {
			allRoads.pop();
			if (allRoads.empty()) {
				cout << "no path" << endl;
				memcpy(map, (uchar*)calloc(width*height, sizeof(uchar)), width*height);
				return;
			}
		}
		A_star(map, width, height, start, goal, allRoads);
	}
}

void onMouse (int event, int x, int y, int flag, void* param) {
	MouseParams* mp = (MouseParams*)param;
	Mat& image = mp->image;
	point* start = mp->start;
	point* goal = mp->goal;
	Mat temp;
	image.copyTo(temp);
	if ((start->x != -1 && start->y != -1) && (goal->x == -1 && goal->y == -1))
		circle(temp, Point(start->x, start->y), 4, Scalar(0, 255, 0), FILLED, LINE_AA);
	else if ((start->x == -1 && start->y == -1) && (goal->x != -1 && goal->y != -1))
		circle(temp, Point(goal->x, goal->y), 4, Scalar(255, 0, 0), FILLED, LINE_AA);
	else if ((start->x != -1 && start->y != -1) && (goal->x != -1 && goal->y != -1)) {
		circle(temp, Point(start->x, start->y), 4, Scalar(0, 255, 0), FILLED, LINE_AA);
		circle(temp, Point(goal->x, goal->y), 4, Scalar(255, 0, 0), FILLED, LINE_AA);
	}

	if (event==EVENT_FLAG_LBUTTON && image.at<Vec3b>(y, x)[0])
		*start = {x, y};
	if (event==EVENT_FLAG_RBUTTON && image.at<Vec3b>(y, x)[0])
		*goal = {x, y};

	imshow("Mark", temp);
}


int main()
{
	Mat image = imread("image/map.png", 0);
	uchar* map = Mat2Array(image);
	int height=image.rows, width=image.cols;

	// select start and goal point
	Mat markMap;
	image.copyTo(markMap);
	cvtColor(markMap, markMap, COLOR_GRAY2BGR);
	point start, goal;
	MouseParams mp = {markMap, &start, &goal};
	namedWindow("Mark");
	moveWindow("Mark", 150, 150);
	imshow("Mark", markMap);
	setMouseCallback("Mark", onMouse, (void*)(&mp));
	waitKey(0);
	destroyWindow("Mark");

	cout << "Click the left/right mouse button to set the starting/end point." << endl;
	cout << "Press any button to complete the setting." << endl << endl;

	if (start.x==-1 || start.y==-1 || goal.x==-1 || goal.y==-1) {
		cout << "Please select starting and end point." << endl;
		exit(0);
	}

	// thinning map
	uchar* thinMap = (uchar*)malloc(width*height*sizeof(uchar));
	uchar* thinMap_ = (uchar*)malloc(width*height*sizeof(uchar));
	memcpy(thinMap, map, width*height);
	memcpy(thinMap_, map, width*height);
	bool convergence;
	do {
		convergence = true;
		thinMap = thinning(thinMap, width, height);
		for (int index=0; index<width*height; index++) {
			convergence = convergence && (thinMap_[index] == thinMap[index]);
			if (!convergence) break;
		}
		memcpy(thinMap_, thinMap, width*height);
	} while (!convergence);
	
	// find new point in thinning map
	point newStart = start;
	point newGoal = goal;
	int MAX_LENGTH = 10;
	// find nearest start point
	for (int i=0; i<MAX_LENGTH; i++) {
		int left = max(start.x-i, 0);
		int right = min(start.x+i, width-1);
		int up = max(start.y-i, 0);
		int down = min(start.y+i, height-1);
		// up
		if (thinMap[up*width+start.x] && map[up*width+start.x]) {
			newStart.y = up;
			break;
		}
		// down
		if (thinMap[down*width+start.x] && map[down*width+start.x]) {
			newStart.y = down;
			break;
		}
		// right
		if (thinMap[start.y*width+right] && map[start.y*width+right]) {
			newStart.x = right;
			break;
		}
		// left
		if (thinMap[start.y*width+left] && map[start.y*width+left]) {
			newStart.x = left;
			break;
		}
	}
	// find nearest goal point 
	for (int i=0; i<MAX_LENGTH; i++) {
		int left = max(goal.x-i, 0);
		int right = min(goal.x+i, width-1);
		int up = max(goal.y-i, 0);
		int down = min(goal.y+i, height-1);
		// up
		if (thinMap[up*width+goal.x] && map[up*width+goal.x]) {
			newGoal.y = up;
			break;
		}
		// down
		if (thinMap[down*width+goal.x] && map[down*width+goal.x]) {
			newGoal.y = down;
			break;
		}
		// right
		if (thinMap[goal.y*width+right] && map[goal.y*width+right]) {
			newGoal.x = right;
			break;
		}
		// left
		if (thinMap[goal.y*width+left] && map[goal.y*width+left]) {
			newGoal.x = left;
			break;
		}
	}
	

	// find sortest path
	A_star(thinMap, width, height, newStart, newGoal);
	Mat path(height, width, CV_8UC1, thinMap);
	// visualization
	for (int index=0; index<width*height; index++)
		if (!map[index])
			map[index] = 128;
		else
			map[index] = 255;
	Mat mapPath(height, width, CV_8UC1, map);
	cvtColor(mapPath, mapPath, COLOR_GRAY2BGR);
	for (int row=0; row<height; row++) {
		for (int col=0; col<width; col++) {
			if (path.at<uchar>(row, col)) {
				mapPath.at<Vec3b>(row, col)[0] = 0;
				mapPath.at<Vec3b>(row, col)[1] = 0;
				mapPath.at<Vec3b>(row, col)[2] = 255;
			}
		}
	}

	// mark start and goal point
	circle(mapPath, Point(start.x, start.y), 4, Scalar(0, 255, 0), FILLED, LINE_AA);
	circle(mapPath, Point(goal.x, goal.y), 4, Scalar(255, 0, 0), FILLED, LINE_AA);

	imshow("shortest path", mapPath);
	waitKey();
	destroyAllWindows();
}