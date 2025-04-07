#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Vec3 origin;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}

};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		float denom = n.dot(r.dir); // 计算 n · d

		// 若 denom 近似为 0，则光线与平面平行
		if (fabs(denom) < 1e-6)
			return false;

		// 计算交点的 t 值
		t = d -(n.dot(r.o)) / denom;

		// 如果 t < 0，说明交点在光线的反方向，返回 false
		return t >= 0;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();  // generate random number r1
		float r2 = sampler->next();  // generate random number r2

		// convert (r1, r2) to vertices of triangle
		float sqrtR1 = sqrt(r1);
		float u = 1.0f - sqrtR1;
		float v = r2 * sqrtR1;

		Vec3 sampledPoint = vertices[0].p * u + vertices[1].p * v + vertices[2].p * (1 - u - v); // interplate coordination
		pdf = 1.0f / area;  // compute PDF (uniform distribution)
		return sampledPoint;
	}


	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	const {
		float t0 = 0.0f;
		float t1 = FLT_MAX;

		// X 轴
		if (fabs(r.dir.x) < 1e-8f) {
			if (r.o.x < min.x || r.o.x > max.x) return false;
		}
		else {
			float invDx = 1.0f / r.dir.x;
			float tx0 = (min.x - r.o.x) * invDx;
			float tx1 = (max.x - r.o.x) * invDx;
			if (tx0 > tx1) std::swap(tx0, tx1);
			t0 = std::max(t0, tx0);
			t1 = std::min(t1, tx1);
			if (t0 > t1) return false;
		}

		// Y 轴
		if (fabs(r.dir.y) < 1e-8f) {
			if (r.o.y < min.y || r.o.y > max.y) return false;
		}
		else {
			float invDy = 1.0f / r.dir.y;
			float ty0 = (min.y - r.o.y) * invDy;
			float ty1 = (max.y - r.o.y) * invDy;
			if (ty0 > ty1) std::swap(ty0, ty1);
			t0 = std::max(t0, ty0);
			t1 = std::min(t1, ty1);
			if (t0 > t1) return false;
		}

		// Z 轴
		if (fabs(r.dir.z) < 1e-8f) {
			if (r.o.z < min.z || r.o.z > max.z) return false;
		}
		else {
			float invDz = 1.0f / r.dir.z;
			float tz0 = (min.z - r.o.z) * invDz;
			float tz1 = (max.z - r.o.z) * invDz;
			if (tz0 > tz1) std::swap(tz0, tz1);
			t0 = std::max(t0, tz0);
			t1 = std::min(t1, tz1);
			if (t0 > t1) return false;
		}

		// 走到这里说明区间 [t0, t1] 有效
		// 我们一般把 t0 当作射线最近的可见碰撞点
		if (t1 < 0.0f) {
			// 整个区间都在光线后面，无交点
			return false;
		}

		// 若 t0 < 0，可能表示射线起点在盒子内
		// 那么可见的第一个交点实际上是 t1
		if (t0 < 0.0f)
		{
			t = t1;
			if (t < 0.0f) return false; // double-check
		}
		else
		{
			t = t0;
		}
		return true;
	}

	// Add code here
	bool rayAABB(const Ray& r)
	const {
		float t0 = 0.0f;          // 光线起点可视作 t=0
		float t1 = FLT_MAX;       // 最大可取值

		// 在 x 轴方向上的进入/离开
		if (fabs(r.dir.x) < 1e-8f) {
			// 光线在 x 方向近似平行, 如果起点在盒子外, 则无交点
			if (r.o.x < min.x || r.o.x > max.x) return false;
		}
		else {
			float invDx = 1.0f / r.dir.x;
			float tx0 = (min.x - r.o.x) * invDx;
			float tx1 = (max.x - r.o.x) * invDx;
			if (tx0 > tx1) std::swap(tx0, tx1);
			t0 = std::max(t0, tx0);    // 取进入最大值
			t1 = std::min(t1, tx1);    // 取离开最小值
			if (t0 > t1) return false; // 若区间反转，说明无效
		}

		// 在 y 轴方向上的进入/离开
		if (fabs(r.dir.y) < 1e-8f) {
			if (r.o.y < min.y || r.o.y > max.y) return false;
		}
		else {
			float invDy = 1.0f / r.dir.y;
			float ty0 = (min.y - r.o.y) * invDy;
			float ty1 = (max.y - r.o.y) * invDy;
			if (ty0 > ty1) std::swap(ty0, ty1);
			t0 = std::max(t0, ty0);
			t1 = std::min(t1, ty1);
			if (t0 > t1) return false;
		}

		// 在 z 轴方向上的进入/离开
		if (fabs(r.dir.z) < 1e-8f) {
			if (r.o.z < min.z || r.o.z > max.z) return false;
		}
		else {
			float invDz = 1.0f / r.dir.z;
			float tz0 = (min.z - r.o.z) * invDz;
			float tz1 = (max.z - r.o.z) * invDz;
			if (tz0 > tz1) std::swap(tz0, tz1);
			t0 = std::max(t0, tz0);
			t1 = std::min(t1, tz1);
			if (t0 > t1) return false;
		}

		// 能执行到这，说明有交集区间 [t0, t1]
		// 只要 t1 >= 0，就说明有在前向方向的交点
		return (t1 >= 0.0f);
	}

	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 oc = r.o - centre;

		// A, B, C 对应二次方程的系数
		float A = r.dir.dot(r.dir);
		float B = 2.0f * oc.dot(r.dir);
		float C = oc.dot(oc) - radius * radius;

		// 判别式
		float discriminant = B * B - 4.0f * A * C;
		if (discriminant < 0.0f)
		{
			// 没有实数解 -> 不相交
			return false;
		}

		// 有解则取更小的正根
		float sqrtD = sqrtf(discriminant);
		float t1 = (-B - sqrtD) / (2.0f * A);
		float t2 = (-B + sqrtD) / (2.0f * A);

		// 两个解都小于 0，表示交点在光线反方向
		if (t1 < 0.0f && t2 < 0.0f)
			return false;

		// 如果 t1 < 0 而 t2 >= 0，表示射线从球内向外打
		if (t1 < 0.0f)
		{
			t = t2; // 只有 t2 有效
			return true;
		}
		else
		{
			// t1 >= 0
			// 如果 t1 < t2，就用 t1；否则用 t2
			// (大部分情况下 t1 < t2)
			t = (t1 < t2) ? t1 : t2;
			return true;
		}
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

//class BVHNode
//{
//public:
//	AABB bounds;
//	BVHNode* r;
//	BVHNode* l;
//
//	unsigned int offset;
//	unsigned char num;
//	BVHNode()
//	{
//		r = NULL;
//		l = NULL;
//		offset = 0;
//		num = 0;
//	}
//	// Note there are several options for how to implement the build method. Update this as required
//	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& orderedTriangles) {
//		// 1. 计算当前节点的包围盒
//		bounds.reset();
//		for (const Triangle& tri : inputTriangles) {
//			bounds.extend(tri.vertices[0].p);
//			bounds.extend(tri.vertices[1].p);
//			bounds.extend(tri.vertices[2].p);
//		}
//
//		// 2. 如果三角形数量小于等于阈值，创建叶子节点
//		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
//			offset = orderedTriangles.size();
//			num = static_cast<unsigned char>(inputTriangles.size());
//			orderedTriangles.insert(orderedTriangles.end(), inputTriangles.begin(), inputTriangles.end());
//			return;
//		}
//
//		// 3. 使用SAH找到最佳分割
//		int bestAxis = -1;
//		int bestSplit = -1;
//		float bestCost = std::numeric_limits<float>::max();
//		Vec3 boundsSize = bounds.max - bounds.min;
//
//		// 尝试所有三个轴
//		for (int axis = 0; axis < 3; axis++) {
//			// 跳过尺寸太小的轴
//			if (boundsSize[axis] < 1e-4f) continue;
//
//			// 创建分割桶
//			float boundsMin = bounds.min[axis];
//			float boundsMax = bounds.max[axis];
//			float binSize = (boundsMax - boundsMin) / BUILD_BINS;
//
//			std::vector<AABB> binBounds(BUILD_BINS);
//			std::vector<int> binCounts(BUILD_BINS, 0);
//
//			// 填充桶
//			for (const Triangle& tri : inputTriangles) {
//				Vec3 center = tri.centre();
//				int binIdx = std::min(BUILD_BINS - 1,
//					static_cast<int>((center[axis] - boundsMin) / binSize));
//				binBounds[binIdx].extend(tri.vertices[0].p);
//				binBounds[binIdx].extend(tri.vertices[1].p);
//				binBounds[binIdx].extend(tri.vertices[2].p);
//				binCounts[binIdx]++;
//			}
//
//			// 评估所有可能的分割
//			for (int split = 1; split < BUILD_BINS; split++) {
//				AABB leftBounds, rightBounds;
//				int leftCount = 0, rightCount = 0;
//
//				// 计算左侧属性
//				for (int i = 0; i < split; i++) {
//					leftBounds.extend(binBounds[i].min);
//					leftBounds.extend(binBounds[i].max);
//					leftCount += binCounts[i];
//				}
//
//				// 计算右侧属性
//				for (int i = split; i < BUILD_BINS; i++) {
//					rightBounds.extend(binBounds[i].min);
//					rightBounds.extend(binBounds[i].max);
//					rightCount += binCounts[i];
//				}
//
//				// 计算SAH成本
//				float leftArea = leftBounds.area();
//				float rightArea = rightBounds.area();
//				float totalArea = bounds.area();
//				float cost = TRAVERSE_COST +
//					(leftCount * leftArea * TRIANGLE_COST +
//						rightCount * rightArea * TRIANGLE_COST) / totalArea;
//
//				if (cost < bestCost) {
//					bestCost = cost;
//					bestAxis = axis;
//					bestSplit = split;
//				}
//			}
//		}
//
//		// 4. 如果没有找到好的分割，创建叶子节点
//		float leafCost = TRIANGLE_COST * inputTriangles.size();
//		if (bestAxis == -1 || bestCost >= leafCost) {
//			offset = orderedTriangles.size();
//			num = static_cast<unsigned char>(inputTriangles.size());
//			orderedTriangles.insert(orderedTriangles.end(), inputTriangles.begin(), inputTriangles.end());
//			return;
//		}
//
//		// 5. 根据最佳分割划分三角形
//		float boundsMin = bounds.min[bestAxis];
//		float boundsMax = bounds.max[bestAxis];
//		float binSize = (boundsMax - boundsMin) / BUILD_BINS;
//		float splitPos = boundsMin + bestSplit * binSize;
//
//		std::vector<Triangle> leftTriangles, rightTriangles;
//		for (Triangle& tri : inputTriangles) {
//			Vec3 center = tri.centre();
//			if (center[bestAxis] < splitPos) {
//				leftTriangles.push_back(tri);
//			}
//			else {
//				rightTriangles.push_back(tri);
//			}
//		}
//
//		// 6. 如果分割失败（所有三角形都在一侧），创建叶子节点
//		if (leftTriangles.empty() || rightTriangles.empty()) {
//			offset = orderedTriangles.size();
//			num = static_cast<unsigned char>(inputTriangles.size());
//			orderedTriangles.insert(orderedTriangles.end(), inputTriangles.begin(), inputTriangles.end());
//			return;
//		}
//
//		// 7. 递归构建子树
//		l = new BVHNode();
//		r = new BVHNode();
//		l->build(leftTriangles, orderedTriangles);
//		r->build(rightTriangles, orderedTriangles);
//	}
//	
//	bool traverse(const Ray& ray, const std::vector<Triangle>& triangles,
//		IntersectionData& intersection) const
//	{
//		// 1. 检查射线是否与当前节点的AABB相交
//		if (!bounds.rayAABB(ray)) {
//			return false;
//		}
//
//		bool hit = false;
//
//		// 2. 如果是叶子节点，检查所有三角形
//		if (l == nullptr && r == nullptr) {
//			for (unsigned int i = 0; i < num; i++) {
//				const Triangle& tri = triangles[offset + i];
//				float u, v, t;
//
//				if (tri.rayIntersect(ray, t, u, v)) {
//					if (t < intersection.t && t > 0.0f) {
//						intersection.t = t;
//						intersection.ID = offset + i;
//						intersection.alpha = 1.0f - u - v;
//						intersection.beta = u;
//						intersection.gamma = v;
//						hit = true;
//					}
//				}
//			}
//			return hit;
//		}
//
//		// 3. 如果是内部节点，递归检查子节点
//		IntersectionData leftIntersection, rightIntersection;
//		bool hitLeft = false;
//		bool hitRight = false;
//
//		// 检查左子树
//		if (l != nullptr) {
//			hitLeft = l->traverse(ray, triangles, intersection);
//		}
//
//		// 检查右子树
//		if (r != nullptr) {
//			hitRight = r->traverse(ray, triangles, intersection);
//		}
//
//		return hitLeft || hitRight;
//	}
//	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
//	{
//		IntersectionData intersection;
//		intersection.t = FLT_MAX;
//		traverse(ray, triangles, intersection);
//		return intersection;
//	}
//	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT) const
//	{
//		// 1. 首先检查射线是否与当前节点的AABB相交
//		// 使用不需要返回t的版本，因为我们只需要知道是否相交
//		if (!bounds.rayAABB(ray)) {
//			return true; // 未相交，继续检查其他节点
//		}
//
//		// 2. 如果是叶子节点，检查所有三角形
//		if (l == nullptr && r == nullptr) {
//			for (unsigned int i = 0; i < num; i++) {
//				const Triangle& tri = triangles[offset + i];
//				float u, v, t;
//
//				if (tri.rayIntersect(ray, t, u, v)) {
//					// 检查交点是否在有效范围内 (EPSILON < t < maxT)
//					if (t > EPSILON && t < maxT) {
//						return false; // 发现阻挡，立即返回不可见
//					}
//				}
//			}
//			return true; // 当前节点内无阻挡
//		}
//
//		// 3. 如果是内部节点，递归检查子节点
//		// 先检查左子树，如果发现阻挡立即返回
//		if (l != nullptr && !l->traverseVisible(ray, triangles, maxT)) {
//			return false;
//		}
//		// 再检查右子树，如果发现阻挡立即返回
//		if (r != nullptr && !r->traverseVisible(ray, triangles, maxT)) {
//			return false;
//		}
//
//		return true; // 所有子节点都无阻挡
//	}
//};
class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	unsigned int offset;  // 三角形列表中的起始索引
	unsigned char num;    // 该节点包含的三角形数量
	bool isLeaf;         // 是否是叶子节点

	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
		isLeaf = false;
	}

	~BVHNode()
	{
		if (r) delete r;
		if (l) delete l;
	}
#if 0
	// Version without SAH
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		// 如果是叶子节点
		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			isLeaf = true;
			offset = (unsigned int)triangles.size();  // 设置三角形的起始索引
			num = (unsigned char)inputTriangles.size();

			// 将三角形添加到全局三角形列表中
			triangles.insert(triangles.end(), inputTriangles.begin(), inputTriangles.end());

			// 计算包围盒
			bounds.reset();
			for (const auto& triangle : inputTriangles) {
				bounds.extend(triangle.vertices[0].p);
				bounds.extend(triangle.vertices[1].p);
				bounds.extend(triangle.vertices[2].p);
			}
			return;
		}

		// 计算整个节点的包围盒
		bounds.reset();
		for (const auto& triangle : inputTriangles) {
			bounds.extend(triangle.vertices[0].p);
			bounds.extend(triangle.vertices[1].p);
			bounds.extend(triangle.vertices[2].p);
		}
		// SAH
		const int BUCKETS = 12;
		const float C_trav = 1.0f;
		const float C_intersect = 2.0f;
		float minCost = FLT_MAX;
		int bestAxis = -1;
		float bestSplit = 0;


		// TODO USE SAH TO BOOST THE PERFORMANCE
		// 计算所有三角形的中心点
		std::vector<Vec3> centers;
		centers.reserve(inputTriangles.size());
		for (const auto& triangle : inputTriangles) {
			centers.push_back(triangle.centre());
		}

		// 选择最佳分割轴
		Vec3 extent = bounds.max - bounds.min;
		int axis = 0;
		if (extent.y > extent.x && extent.y > extent.z) axis = 1;
		else if (extent.z > extent.x && extent.z > extent.y) axis = 2;

		// 计算分割点（使用中位数）
		float split = 0.0f;
		std::vector<float> centerValues(centers.size());
		for (size_t i = 0; i < centers.size(); i++) {
			centerValues[i] = (axis == 0) ? centers[i].x : ((axis == 1) ? centers[i].y : centers[i].z);
		}
		size_t mid = centerValues.size() / 2;
		std::nth_element(centerValues.begin(), centerValues.begin() + mid, centerValues.end());
		split = centerValues[mid];

		// 分割三角形
		std::vector<Triangle> leftTriangles, rightTriangles;
		for (size_t i = 0; i < inputTriangles.size(); i++) {
			float centerValue = (axis == 0) ? centers[i].x : ((axis == 1) ? centers[i].y : centers[i].z);
			if (centerValue <= split) {
				leftTriangles.push_back(inputTriangles[i]);
			}
			else {
				rightTriangles.push_back(inputTriangles[i]);
			}
		}

		// 处理特殊情况：如果一边为空，则平均分配
		if (leftTriangles.empty() || rightTriangles.empty()) {
			size_t mid = inputTriangles.size() / 2;
			leftTriangles.assign(inputTriangles.begin(), inputTriangles.begin() + mid);
			rightTriangles.assign(inputTriangles.begin() + mid, inputTriangles.end());
		}

		// 递归构建左右子树
		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTriangles, triangles);
		r->build(rightTriangles, triangles);
	}
#endif
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		// 如果是叶子节点
		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			isLeaf = true;
			offset = (unsigned int)triangles.size();  // 设置三角形的起始索引
			num = (unsigned char)inputTriangles.size();

			// 将三角形添加到全局三角形列表中
			triangles.insert(triangles.end(), inputTriangles.begin(), inputTriangles.end());

			// 计算包围盒
			bounds.reset();
			for (const auto& triangle : inputTriangles) {
				bounds.extend(triangle.vertices[0].p);
				bounds.extend(triangle.vertices[1].p);
				bounds.extend(triangle.vertices[2].p);
			}
			return;
		}

		bounds.reset();
		for (const auto& triangle : inputTriangles) {
			bounds.extend(triangle.vertices[0].p);
			bounds.extend(triangle.vertices[1].p);
			bounds.extend(triangle.vertices[2].p);
		}

		// SAH参数（同原代码）
		const int BUCKETS = 12;
		const float C_trav = 1.0f;
		const float C_intersect = 2.0f;
		float minCost = FLT_MAX;
		int bestAxis = -1;
		float bestSplit = 0;

		// 遍历三个轴（X/Y/Z）
		for (int axis = 0; axis < 3; ++axis) {
			// 改用索引避免指针失效
			std::vector<std::pair<float, size_t>> sortedCenters;
			sortedCenters.reserve(inputTriangles.size());
			for (size_t i = 0; i < inputTriangles.size(); ++i) {
				Vec3 center = inputTriangles[i].centre();
				sortedCenters.emplace_back(center[axis], i);
			}
			std::sort(sortedCenters.begin(), sortedCenters.end());

			// 为每个桶计算候选分割点
			for (int b = 1; b < BUCKETS; ++b) {
				size_t splitIndex = (b * sortedCenters.size()) / BUCKETS;
				if (splitIndex == 0 || splitIndex >= sortedCenters.size()) continue;

				// 分割点值
				float splitValue = sortedCenters[splitIndex].first;

				// 计算左右包围盒（初始化 leftBox 和 rightBox）
				AABB leftBox, rightBox;
				leftBox.reset();
				rightBox.reset();

				for (size_t i = 0; i < splitIndex; ++i) {
					const Triangle& tri = inputTriangles[sortedCenters[i].second];
					leftBox.extend(tri.vertices[0].p);
					leftBox.extend(tri.vertices[1].p);
					leftBox.extend(tri.vertices[2].p);
				}
				for (size_t i = splitIndex; i < sortedCenters.size(); ++i) {
					const Triangle& tri = inputTriangles[sortedCenters[i].second];
					rightBox.extend(tri.vertices[0].p);
					rightBox.extend(tri.vertices[1].p);
					rightBox.extend(tri.vertices[2].p);
				}

				// 计算SAH成本
				float cost = C_trav +
					(leftBox.area() * splitIndex + rightBox.area() * (sortedCenters.size() - splitIndex)) * C_intersect / bounds.area();

				if (cost < minCost) {
					minCost = cost;
					bestAxis = axis;
					bestSplit = splitValue;
				}
			}
		}

		// 按最佳轴和分割点划分三角形
		std::vector<Triangle> leftTriangles, rightTriangles;
		for (auto& tri : inputTriangles) {
			float centerValue = tri.centre()[bestAxis];
			if (centerValue <= bestSplit) {
				leftTriangles.push_back(tri);
			}
			else {
				rightTriangles.push_back(tri);
			}
		}

		// 处理特殊情况：如果一边为空，则平均分配
		if (leftTriangles.empty() || rightTriangles.empty()) {
			size_t mid = inputTriangles.size() / 2;
			leftTriangles.assign(inputTriangles.begin(), inputTriangles.begin() + mid);
			rightTriangles.assign(inputTriangles.begin() + mid, inputTriangles.end());
		}

		// 递归构建左右子树前显式创建子节点
		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTriangles, triangles);
		r->build(rightTriangles, triangles);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// 首先检查射线是否与当前节点的包围盒相交
		float boxT;
		if (!bounds.rayAABB(ray, boxT) || boxT > intersection.t) {
			return;
		}

		// 如果是叶子节点，测试与所有三角形的相交
		if (isLeaf) {
			for (unsigned int i = 0; i < num; i++) {
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v)) {
					if (t > 0 && t < intersection.t) {  // 确保t为正且是最近的交点
						intersection.t = t;
						intersection.ID = offset + i;
						intersection.alpha = 1.0f - (u + v);
						intersection.beta = u;
						intersection.gamma = v;
					}
				}
			}
			return;
		}

		// 递归遍历子节点，先遍历更近的子节点
		float tLeft = FLT_MAX, tRight = FLT_MAX;
		bool hitLeft = l ? l->bounds.rayAABB(ray, tLeft) : false;
		bool hitRight = r ? r->bounds.rayAABB(ray, tRight) : false;

		if (tLeft < tRight) {
			if (hitLeft) l->traverse(ray, triangles, intersection);
			if (hitRight && tRight < intersection.t) r->traverse(ray, triangles, intersection);
		}
		else {
			if (hitRight) r->traverse(ray, triangles, intersection);
			if (hitLeft && tLeft < intersection.t) l->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// 首先检查射线是否与当前节点的包围盒相交
		float t;
		if (!bounds.rayAABB(ray, t))
		{
			return true;
		}

		// 如果是叶子节点,测试与所有三角形的相交
		if (isLeaf)
		{
			for (unsigned int i = 0; i < num; i++)
			{
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					if (t < maxT)
					{
						return false;
					}
				}
			}
			return true;
		}

		// 如果是内部节点,递归遍历左右子树
		if (l && !l->traverseVisible(ray, triangles, maxT)) return false;
		if (r && !r->traverseVisible(ray, triangles, maxT)) return false;
		return true;
	}
};