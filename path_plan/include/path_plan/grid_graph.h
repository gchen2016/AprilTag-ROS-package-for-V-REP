#ifndef GRID_GRAPH_H
#define GRID_GRAPH_H

#include <vector>

namespace path_plan {
	const unsigned COST_LINE = 1;
	const unsigned COST_ROTATE = 1;
	const int DIRECTION = 4;
	const int FLOOR_LENGTH = 5;

	enum Direction {
		DOWN,
		LEFT,
		UP,
		RIGHT
	};

	struct Node;

	struct Edge {
		Node *orgNode; //the originating vertex
		Node *dstNode; //the destination vertex
		unsigned cost; //cost of the edge

		Edge(Node *firstNode, Node *secNode, unsigned inCost) : orgNode(firstNode), dstNode(secNode), cost(inCost){};

	};

	struct Node {
		int x;
		int y;
		int theta;
		int id;
		std::vector<Edge> edgeList; //list of outgoing edges for this vertex

		Node(int inputX, int inputY, int inputT, int inputId) : x(inputX), y(inputY), theta(inputT), id(inputId){};

		//do not del the adj nodes here...they will be deleted by graph destructor
		~Node() {
			edgeList.clear();
		}

		void addAdjNode(Node **adj, unsigned cost) {
		//create an edge with 'this' as the originating node and adj as the destination node
			Edge newEdge(this, *adj, cost);
			edgeList.push_back(newEdge);
		}

	};

	//An object of class graph holds a directed graph
	struct GridGraph {
		int l;
		std::vector<Node*> nodeList; //list of verticies

		GridGraph() : l(FLOOR_LENGTH) {
			// construct nodes
			for(int x=0; x < l; x++) {
				for(int y=0; y < l; y++) {
					for(int t=0; t < DIRECTION; t++) {
						Node *tmpNode = new Node(x, y, t, x*l*DIRECTION + y*DIRECTION + t);
						nodeList.push_back(tmpNode);
					}
				}
			}

			// construct edges
			for(int x=0; x < l; x++) {
				unsigned cost;
				for(int y=0; y < l; y++) {
					for(int t=0; t < DIRECTION; t++) {
						int id = x*l*DIRECTION + y*DIRECTION + t;
						int id_base = x*l*DIRECTION + y*DIRECTION;

						// adj theta
						cost = COST_ROTATE;
						switch (t) {
							case UP:
								nodeList[id]->addAdjNode(&nodeList[id_base + LEFT], COST_ROTATE);
								nodeList[id]->addAdjNode(&nodeList[id_base + RIGHT], COST_ROTATE);
								break;
							case RIGHT:
								nodeList[id]->addAdjNode(&nodeList[id_base + UP], COST_ROTATE);
								nodeList[id]->addAdjNode(&nodeList[id_base + DOWN], COST_ROTATE);
								break;
							case DOWN:
								nodeList[id]->addAdjNode(&nodeList[id_base + RIGHT], COST_ROTATE);
								nodeList[id]->addAdjNode(&nodeList[id_base + LEFT], COST_ROTATE);
								break;
							case LEFT:
								nodeList[id]->addAdjNode(&nodeList[id_base + DOWN], COST_ROTATE);
								nodeList[id]->addAdjNode(&nodeList[id_base + UP], COST_ROTATE);
								break;
						}

						cost = COST_LINE;
						// top bound
						if(x > 0 && t == UP) 
							nodeList[id]->addAdjNode(&nodeList[(x-1)*l*DIRECTION + y*DIRECTION + t], COST_LINE);
						// left bound
						if(y > 0 && t == LEFT) 
							nodeList[id]->addAdjNode(&nodeList[x*l*DIRECTION + (y-1)*DIRECTION + t], COST_LINE);
						// lower bound
						if(x < (l-1) && t == DOWN)
							nodeList[id]->addAdjNode(&nodeList[(x+1)*l*DIRECTION + y*DIRECTION + t], COST_LINE);
						// right bound
						if(y < (l-1) && t == RIGHT)
							nodeList[id]->addAdjNode(&nodeList[x*l*DIRECTION + (y+1)*DIRECTION + t], COST_LINE);
					}
					
					
				}
			}
		}

		~GridGraph() {
		//free mem allocated to verticies
			for(int i=0; i < nodeList.size(); i++)
				delete nodeList[i];
			nodeList.clear();
		}
	};
}

#endif