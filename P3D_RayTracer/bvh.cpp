#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB &bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_)
{
	this->leaf = true;
	this->index = index_;
	this->n_objs = n_objs_;
}

void BVH::BVHNode::makeNode(unsigned int left_index_)
{
	this->leaf = false;
	this->index = left_index_;
	// this->n_objs = n_objs_;
}

BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }

void BVH::Build(vector<Object *> &objs)
{
	BVHNode *root = new BVHNode();

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);

	for (Object *obj : objs)
	{
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON;
	world_bbox.min.y -= EPSILON;
	world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON;
	world_bbox.max.y += EPSILON;
	world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node)
{
	if ((right_index - left_index) <= Threshold)
	{
		node->makeLeaf(left_index, right_index - left_index);
		return;
	}

	// Find the largest AABBs extent
	float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
	float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;

	int diffX = node->getAABB().max.x - node->getAABB().min.x;
	int diffY = node->getAABB().max.y - node->getAABB().min.y;
	int diffZ = node->getAABB().max.z - node->getAABB().min.z;

	// for (int i = left_index; i < right_index; i++)
	// {
	// 	AABB bbox = objects[i]->GetBoundingBox();
	// 	Vector centroid = bbox.centroid();

	// 	if (centroid.x > maxX)
	// 		maxX = centroid.x;

	// 	if (centroid.y > maxY)
	// 		maxY = centroid.y;

	// 	if (centroid.z > maxZ)
	// 		maxZ = centroid.z;

	// 	if (centroid.x < minX)
	// 		minX = centroid.x;

	// 	if (centroid.y < minY)
	// 		minY = centroid.y;

	// 	if (centroid.z < minZ)
	// 		minZ = centroid.z;
	// }

	// float diffX = maxX - minX, diffY = maxY - minY, diffZ = maxZ - minZ;
	int axis = (diffX >= diffY && diffX >= diffZ) ? 0 : (diffY >= diffZ) ? 1 : 2;

	Comparator cmp;
	cmp.dimension = axis;
	sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

	// Find the split index
	int split_index = (left_index + right_index) / 2;

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
	Vector max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	AABB lBox = AABB(min, max);
	AABB rBox = AABB(min, max);

	// Calculate bounding boxes of left and right sides
	for (int i = left_index; i < split_index; i++)
	{
		AABB bbox = objects[i]->GetBoundingBox();
		lBox.extend(bbox);
	}

	for (int i = split_index; i < right_index; i++)
	{
		AABB bbox = objects[i]->GetBoundingBox();
		rBox.extend(bbox);
	}

	lBox.min.x -= EPSILON, lBox.min.y -= EPSILON, lBox.min.z -= EPSILON;
	lBox.max.x += EPSILON, lBox.max.y += EPSILON, lBox.max.z += EPSILON;

	rBox.min.x -= EPSILON, rBox.min.y -= EPSILON, rBox.min.z -= EPSILON;
	rBox.max.x += EPSILON, rBox.max.y += EPSILON, rBox.max.z += EPSILON;

	// Create two new nodes
	BVHNode *lNode = new BVHNode();
	BVHNode *rNode = new BVHNode();

	lNode->setAABB(lBox);
	rNode->setAABB(rBox);

	// Initiate current node as an interior node with leftNode and rightNode as children
	node->makeNode(nodes.size());

	nodes.push_back(lNode);
	nodes.push_back(rNode);

	build_recursive(left_index, split_index, lNode);
	build_recursive(split_index, right_index, rNode);
}

bool BVH::Traverse(Ray &ray, Object **hit_obj, Vector &hit_point)
{
	float tClosest = FLT_MAX, tLeft, tRight, tmp; // contains the closest primitive intersection
	bool hitLeft = false, hitRight = false, hit = false;
	Object *closestHit = NULL;

	BVHNode *currentNode = nodes[0];

	// Check if we hit the world bounding box
	if (!currentNode->getAABB().intercepts(ray, tmp))
	{
		return false;
	}

	while (true)
	{
		if (!currentNode->isLeaf())
		{
			BVHNode *leftNode = nodes[currentNode->getIndex()];
			BVHNode *rightNode = nodes[currentNode->getIndex() + 1];

			bool hitLeft = leftNode->getAABB().intercepts(ray, tLeft);
			bool hitRight = rightNode->getAABB().intercepts(ray, tRight);

			if (hitLeft && hitRight)
			{
				if (tLeft < tRight)
				{
					hit_stack.push(StackItem(rightNode, tRight));
					currentNode = leftNode;
				}
				else
				{
					hit_stack.push(StackItem(leftNode, tLeft));
					currentNode = rightNode;
				}
				continue;
			}
			else if (hitLeft || hitRight)
			{
				currentNode = hitLeft ? leftNode : rightNode;
				continue;
			}
		}
		else
		{
			for (int i = currentNode->getIndex(); i < (currentNode->getIndex() + currentNode->getNObjs()); i++)
			{
				if (objects[i]->intercepts(ray, tmp) && tmp < tClosest)
				{
					tClosest = tmp;
					closestHit = objects[i];
				}
			}
		}

		while (!hit_stack.empty())
		{
			StackItem item = hit_stack.top();
			hit_stack.pop();

			if (item.t < tClosest)
			{
				currentNode = item.ptr;
				break;
			}
		}

		if (hit_stack.empty())
		{
			if (closestHit != nullptr)
			{
				*hit_obj = closestHit;
				hit_point = ray.origin + ray.direction * tClosest;
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	return false;
}

bool BVH::Traverse(Ray &ray) // shadow ray with length
{
	float tmp;
	bool hitLeft = false, hitRight = false;
	double length = ray.direction.length(); // distance between light and intersection point
	ray.direction.normalize();

	BVHNode *currentNode = nodes[0];

	if (!currentNode->getAABB().intercepts(ray, tmp))
	{
		return false;
	}

	while (true)
	{
		if (!currentNode->isLeaf())
		{
			BVHNode *leftNode = nodes[currentNode->getIndex()];
			BVHNode *rightNode = nodes[currentNode->getIndex() + 1];

			bool hitLeft = leftNode->getAABB().intercepts(ray, tmp);
			bool hitRight = rightNode->getAABB().intercepts(ray, tmp);

			if (hitLeft && hitRight)
			{
				hit_stack.push(StackItem(rightNode, tmp));
				currentNode = leftNode;
				continue;
			}
			else if (hitLeft || hitRight)
			{
				currentNode = hitLeft ? leftNode : rightNode;
				continue;
			}
		}
		else
		{
			for (int i = currentNode->getIndex(); i < (currentNode->getIndex() + currentNode->getNObjs()); i++)
			{
				if (objects[i]->intercepts(ray, tmp) && tmp < length)
				{
					return true;
				}
			}
		}

		if (hit_stack.empty())
		{
			return false;
		}

		StackItem item = hit_stack.top();
		currentNode = item.ptr;
		hit_stack.pop();
	}

	return false;
}
