/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node*& node, int depth, const std::vector<float>& point, int id)
	{
		if(NULL == node)
		{
			node = new Node(point, id);
			return;
		}

		if(point[depth%2] < node->point[depth%2])
		{
			insertHelper(node->left, depth+1, point, id);
		}
		else insertHelper(node->right, depth+1, point, id);
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	void searchHelper(std::vector<int>& ids, Node*& node, int depth, std::vector<float>& target, float distanceTol)
	{
		if(NULL == node)
			return;

		// Check if this node is in a boxed square that is 2 x distanceTol for length, centered around the target point.
		if(target[0] - distanceTol <= node->point[0] && node->point[0] <= target[0] + distanceTol
		   && target[1] - distanceTol <= node->point[1] && node->point[1] <= target[1] + distanceTol)
		{
			float distance_squared = (target[0] - node->point[0])*(target[0] - node->point[0]) + (target[1] - node->point[1])*(target[1] - node->point[1]);
			float distance = sqrt(distance_squared);
			if(distance <= distanceTol)
			{
				ids.emplace_back(node->id);
			}
		}
		
		if(target[depth%2] - distanceTol < node->point[depth%2])
		{
			searchHelper(ids, node->left, depth+1, target, distanceTol);
		}

		if(target[depth%2] + distanceTol > node->point[depth%2])
		{
			searchHelper(ids, node->right, depth+1, target, distanceTol);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, root, 0, target, distanceTol);
		return ids;
	}
	

};




