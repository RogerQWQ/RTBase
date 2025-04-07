#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"
#include <mutex>
std::mutex coutMutex;
#define ENV_SCALE 0.26f
#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};


class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 MIS(const ShadingData&, Sampler*, Colour&, float&) {
		return Vec3(0.0f,0.f,0.0f); 
	}
	virtual bool envornot() = 0;
	virtual Colour Emitted(const Vec3& wi) = 0;
	virtual Ray sampleRay(Sampler* sampler, float& pdf, Colour& outIntensity) = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission = Colour(10.0f, 10.0f, 10.0f);
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const Vec3& wi)
	{
		return triangle->gNormal();
	}
	bool envornot()
	{
		return false;
	}
	
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		//Vec3 wi = Vec3(0, 0, 1);
		//pdf = 1.0f;

		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
	Ray sampleRay(Sampler* sampler, float& pdf, Colour& outIntensity)
	{
		float pdfPos, pdfDir;

		// 1. 从三角形面采样一个点
		Vec3 origin = triangle->sample(sampler, pdfPos);

		// 2. 从法线方向上 cosine 分布采样一个方向
		Vec3 localDir = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		Frame frame;
		frame.fromVector(triangle->gNormal());
		Vec3 dir = frame.toWorld(localDir);

		if (Dot(dir, triangle->gNormal()) <= 0.0f) {
			dir = -dir;
		}
		

		pdfDir = SamplingDistributions::cosineHemispherePDF(localDir);
		pdf = pdfPos * pdfDir;
		
		outIntensity = emission /(pdf + 1e-6f);
		
		return Ray(origin, dir);
	}
	Colour Emitted(const Vec3& wi)
	{
		return emission;
	}

};

class BackgroundColour : public Light
{
public:
	Colour emission = Colour(10.0f, 10.0f, 10.0f);
	Triangle* triangle = NULL;
	
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Ray sampleRay(Sampler* sampler, float& pdf, Colour& outIntensity) override
	{
		// 1. 环境贴图的重要性采样方向
		ShadingData dummy; // 不使用，传默认值
		Vec3 dir = sample(dummy, sampler, outIntensity, pdf); // 使用已有 sample() 函数
		std::cout << "[sampleRay] emission = (" << emission.r << ", " << emission.g << ", " << emission.b << ")" << std::endl;
		// 2. 原点设为 Vec3(0, 0, 0)，表示从世界中心发出
		return Ray(Vec3(0.0f, 0.0f, 0.0f), dir);
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
	bool envornot()
	{
		return false;
	}
	Colour Emitted(const Vec3& wi)
	{
		return emission;
	}
};
class EnvMapSampler {
public:
	int width, height;
	std::vector<float> marginalCDF;
	std::vector<std::vector<float>> conditionalCDF;
	Texture* env;

	Vec3 sample(float u1, float u2, float& pdf)
	{
		// Sample row (v direction) using marginal CDF
		int y = 0;
		for (; y < height - 1; ++y)
		{
			if (u1 < marginalCDF[y]) break;
		}

		// Map back to row probability range
		float v0 = y == 0 ? 0.0f : marginalCDF[y - 1];
		float v1 = marginalCDF[y];
		float dv = (u1 - v0) / (v1 - v0 + 1e-6f);

		// Sample column (u direction) using conditional CDF
		int x = 0;
		for (; x < width - 1; ++x)
		{
			if (u2 < conditionalCDF[y][x]) break;
		}

		float u0 = x == 0 ? 0.0f : conditionalCDF[y][x - 1];
		float u1cdf = conditionalCDF[y][x];
		float du = (u2 - u0) / (u1cdf - u0 + 1e-6f);

		// convert to u,v in [0,1]
		float u = (x + du) / (float)width;
		float v = (y + dv) / (float)height;

		// latitude -> wi
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);

		Vec3 wi = Vec3(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);

		// compute pdf
		float sinT = std::max(sinTheta, 1e-6f);
		float mapPDF = conditionalCDF[y][x] * marginalCDF[y];
		pdf = (width * height * mapPDF) / (2.0f * M_PI * M_PI * sinT);

		return wi;
	}
	
};
class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<float> pdf2D; // 大小 width*height
	std::vector<float> cdf2D; // 同上
	Triangle* triangle = NULL;
	Colour emission = Colour(10.0f, 10.0f, 10.0f);
	int width;
	int height;
	EnvMapSampler sampler;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		width = env->width;
		height = env->height;

		buildDistribution();
	}
	void buildDistribution()
	{
		pdf2D.resize(width * height);
		cdf2D.resize(width * height);

		// 1) 计算未经归一化的“亮度 * sin(theta)”分布
		float sum = 0.0f;
		for (int y = 0; y < height; y++)
		{
			// v \in [0,1], theta = pi * v
			float v = (y + 0.5f) / (float)height;
			float sinTheta = sinf(M_PI * v);

			for (int x = 0; x < width; x++)
			{
				float u = (x + 0.5f) / (float)width;

				// 贴图上该像素的亮度(或 RGB -> Luma)
				float luminance = env->texels[y * width + x].Lum();

				// 这里的权重是 L(u,v)*sin(theta)
				float val = luminance * sinTheta;

				pdf2D[y * width + x] = val;
				sum += val;
			}
		}


		for (int i = 0; i < width * height; i++)
		{
			pdf2D[i] /= sum;
		}

		
		cdf2D[0] = pdf2D[0];
		for (int i = 1; i < width * height; i++)
		{
			cdf2D[i] = cdf2D[i - 1] + pdf2D[i];
		}
	
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;*/
		// 1) 获取随机数
		float Xi1 = sampler->next();
		// Xi1 \in [0,1], 用于在 cdf2D 里查找
		// Xi2 \in [0,1], 这里也可以留给后面插值或者在 uv 上的细分
		float Xi2 = sampler->next();

		// 2) 在cdf中做“逆变换采样” (Inverse Transform Sampling)
		//    这里用一个简单的二分查找找 index
		int idx = binarySearchCDF(cdf2D, Xi1);
		// idx 对应拍平成一维后的像素下标
		int y = idx / width;
		int x = idx % width;

		// 取得对应的像素中心 (u, v)
		float u = (x + 0.5f) / (float)width;
		float v = (y + 0.5f) / (float)height;

		// 3) 转换到球面方向
		float theta = M_PI * v;      // \theta \in [0,\pi]
		float phi = 2.0f * M_PI * u; // \phi   \in [0,2\pi]

		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);

		Vec3 dir = Vec3(sinTheta * cosPhi,  // x
			cosTheta,           // y
			sinTheta * sinPhi); // z

		// 4) 读取颜色
		reflectedColour = env->sample(u, v);

		// 5) 计算 pdf_\Omega(\omega)
		//    pdf2D[idx] 是 p(u,v) ，球面上的 pdf_\Omega(\omega) = p(u,v) / (2*pi^2 * sin(theta))
		//    注意要避免 sin(theta)=0 的数值问题。
		pdf = pdf2D[idx] / (2.0f * M_PI * M_PI * sinTheta);

		return dir;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		// 让 u \in [0,2pi)
		if (u < 0.0f)
			u += 2.0f * M_PI;
		u /= (2.0f * M_PI);

		float v = acosf(wi.y) / M_PI; // \theta = arccos(wi.y)

		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		/*return SamplingDistributions::uniformSpherePDF(wi);*/

		float phi = atan2f(wi.z, wi.x);
		if (phi < 0.f)
			phi += 2.f * M_PI;
		float u = phi / (2.f * M_PI);

		float theta = acosf(wi.y);  // \theta \in [0,\pi]
		float v = theta / M_PI;

		int x = (int)floorf(u * width);
		int y = (int)floorf(v * height);
		x = std::max(0, std::min(width - 1, x));
		y = std::max(0, std::min(height - 1, y));


		float p_uv = pdf2D[y * width + x];


		float sinTheta = sinf(theta);
		if (sinTheta < 1e-6f)
			return 0.f;
		float pdfOmega = p_uv / (2.f * M_PI * M_PI * sinTheta);

		return pdfOmega;
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const Vec3& wi)
	{
		return -wi;
	}
	bool envornot()
	{
		return true;
	}

	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 MIS(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) override
	{
		// === Decide which strategy to sample ===
		bool sampleEnv = sampler->next() < 0.5f;

		if (sampleEnv) {
			// --- Sample environment map ---
			float Xi1 = sampler->next();
			float Xi2 = sampler->next();
			int idx = binarySearchCDF(cdf2D, Xi1);
			int y = idx / width;
			int x = idx % width;

			float u = (x + 0.5f) / width;
			float v = (y + 0.5f) / height;
			float theta = M_PI * v;
			float phi = 2.0f * M_PI * u;

			float sinTheta = sinf(theta);
			Vec3 dir = Vec3(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));

			float p_env = (sinTheta > 1e-6f) ? pdf2D[idx] / (2.0f * M_PI * M_PI * sinTheta) : 0.f;
			float p_area = SamplingDistributions::cosineHemispherePDF(dir);

			float misWeight = p_env / (p_env + p_area + 1e-6f);
			Colour Le = env->sample(u, v);
			Colour f = shadingData.bsdf->evaluate(shadingData, dir);
			float cosTheta2 = std::max(0.f, Dot(shadingData.sNormal, dir));

			reflectedColour = f * Le * cosTheta2 * misWeight / (p_env + 1e-6f);
			pdf = p_env;
			return dir;
		}
		else {
			// --- Sample cosine-weighted direction for area light ---
			float Xi1 = sampler->next();
			float Xi2 = sampler->next();
			Vec3 localDir = SamplingDistributions::cosineSampleHemisphere(Xi1, Xi2);
			Frame frame;
			frame.fromVector(shadingData.sNormal);
			Vec3 dir = frame.toWorld(localDir);

			float p_area = SamplingDistributions::cosineHemispherePDF(localDir);
			float u = atan2f(dir.z, dir.x) / (2.0f * M_PI);
			float v = acosf(dir.y) / M_PI;
			int x = int(u * width);
			if (x < 0) x = 0;
			if (x >= width) x = width - 1;

			int y = int(v * height);
			if (y < 0) y = 0;
			if (y >= height) y = height - 1;

			float p_env = pdf2D[y * width + x] / (2.0f * M_PI * M_PI * sinf(M_PI * v));

			float misWeight = p_area / (p_area + p_env + 1e-6f);
			Colour Le = Colour(10.f, 10.f, 10.f); 
			Colour f = shadingData.bsdf->evaluate(shadingData, dir);
			float cosTheta2 = std::max(0.f, Dot(shadingData.sNormal, dir));

			reflectedColour = f * Le * cosTheta2 * misWeight / (p_area + 1e-6f);
			pdf = p_area;
			return dir;
		}
	}


	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Ray sampleRay(Sampler* sampler, float& pdf, Colour& outIntensity) override
	{
		Vec3 dir = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(dir);
		outIntensity = emission;
		return Ray(Vec3(0, 0, 0), dir); // 从场景中心发出
	}
	Vec3 sampleDirectionFromLight(Sampler* _sampler, float& pdf)
	{
		//Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		//pdf = SamplingDistributions::uniformSpherePDF(wi);
		//return wi;

		float u1 = _sampler->next();
		float u2 = _sampler->next();
		Vec3 wi = sampler.sample(u1, u2, pdf);

		return wi;
	}


	Colour Emitted(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v) * ENV_SCALE;
	}
	private:
		// 简单二分查找，在 cdf 数组中找到 >= Xi 的最小索引
		int binarySearchCDF(const std::vector<float>& cdf, float Xi)
		{
			int left = 0;
			int right = (int)cdf.size() - 1;

			while (left < right)
			{
				int mid = (left + right) >> 1; // /2
				if (cdf[mid] < Xi)
					left = mid + 1;
				else
					right = mid;
			}
			return left;
		}
};

