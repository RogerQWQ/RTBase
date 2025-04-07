#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};
struct Vec2 {
	float x, y;
	Vec2(float _x = 0, float _y = 0) : x(_x), y(_y) {}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// 将r1映射到方位角φ (0到2π)
		float phi = 2.0f * M_PI * r1;

		// 将r2映射到极角θ的余弦 (0到1)
		// 因为对于均匀半球采样，cosθ应该是均匀分布的
		float cosTheta = r2;
		float sinTheta = sqrt(1.0f - cosTheta * cosTheta);

		// 转换为笛卡尔坐标
		float x = sinTheta * cos(phi);
		float y = sinTheta * sin(phi);
		float z = cosTheta;

		return Vec3(x, y, z);
	}
	static float uniformHemispherePDF(const Vec3& wi)
	{
		// 均匀半球采样的 PDF 是 1 / (2π)
		return 1.0f / (2.0f * M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// 1. 在单位圆盘上均匀采样（使用极坐标）
		float radius = sqrt(r1);          // 使用 sqrt(r1) 来补偿圆盘的面积变化
		float theta = 2.0f * M_PI * r2;   // 方位角均匀分布

		// 2. 转换为笛卡尔坐标（在圆盘上）
		float x = radius * cos(theta);
		float y = radius * sin(theta);

		// 3. 投影到半球上（z = sqrt(1 - x² - y²)）
		float z = sqrt(1.0f - x * x - y * y);

		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3& wi)
	{
		// 余弦加权采样的 PDF 是 cosθ / π
		// 确保 wi 在半球内（z ≥ 0），否则 PDF = 0
		return (wi.z > 0.0f) ? (wi.z / M_PI) : 0.0f;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// 计算方位角 φ ∈ [0, 2π]
		float phi = 2.0f * M_PI * r1;

		// 计算极角 θ ∈ [0, π]，并确保均匀分布
		float cosTheta = 2.0f * r2 - 1.0f;  // 映射到 [-1, 1]
		float sinTheta = sqrt(1.0f - cosTheta * cosTheta);

		// 转换为笛卡尔坐标
		float x = sinTheta * cos(phi);
		float y = sinTheta * sin(phi);
		float z = cosTheta;

		return Vec3(x, y, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// 均匀球面采样的 PDF 是 1 / (4π)
		return 1.0f / (4.0f * M_PI);
	}
	static Vec3 cosineHemisphere(const Vec2& uv) {
		float r = sqrt(uv.x);
		float theta = 2.0f * M_PI * uv.y;

		float x = r * cos(theta);
		float y = r * sin(theta);
		float z = sqrt(1.0f - uv.x);

		return Vec3(x, y, z);
	}
};

class TabulatedDistribution {
public:
	// x 值的离散点与对应概率值（PDF）
	std::vector<float> xValues;
	std::vector<float> pdfValues;
	// 累积分布函数 (CDF)
	std::vector<float> cdf;

	// 构造函数：给定 x 值和对应的 PDF 值，构建 CDF 表
	TabulatedDistribution(const std::vector<float>& x, const std::vector<float>& pdf)
		: xValues(x), pdfValues(pdf) {
		buildCDF();
	}

	// 构建 CDF 表，采用梯形积分法
	void buildCDF() {
		size_t n = pdfValues.size();
		cdf.resize(n);
		float accum = 0.0f;
		cdf[0] = 0.0f;  // 起始点 CDF 为 0
		for (size_t i = 1; i < n; i++) {
			// 计算 [x[i-1], x[i]] 区间上的面积 (近似)
			float dx = xValues[i] - xValues[i - 1];
			float area = 0.5f * (pdfValues[i - 1] + pdfValues[i]) * dx;
			accum += area;
			cdf[i] = accum;
		}
		// 归一化 CDF 到 [0, 1]
		for (size_t i = 0; i < n; i++) {
			cdf[i] /= accum;
		}
	}

	// 通过逆变换采样得到样本 x
	float sample(float r) const {
		// 二分查找：找到最小的索引 i 使得 cdf[i] >= r
		auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
		size_t idx = std::distance(cdf.begin(), it);
		if (idx == 0) {
			return xValues.front();
		}
		if (idx >= cdf.size())
			idx = cdf.size() - 1;

		// 为了更精确，可以对相邻区间做线性插值
		float t = (r - cdf[idx - 1]) / (cdf[idx] - cdf[idx - 1]);
		float x_sample = xValues[idx - 1] + t * (xValues[idx] - xValues[idx - 1]);
		return x_sample;
	}

	// 获取某个 x 对应的 PDF
	float getPDF(float x) const {
		// 这里简单查找最近的离散点（实际可以插值）
		auto it = std::lower_bound(xValues.begin(), xValues.end(), x);
		size_t idx = std::distance(xValues.begin(), it);
		if (idx >= pdfValues.size())
			idx = pdfValues.size() - 1;
		return pdfValues[idx];
	}
};