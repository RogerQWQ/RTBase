#pragma once


#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include <algorithm>
#include <Windows.h>
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <OpenImageDenoise/oidn.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include<string>
#include <typeinfo>
#include <mutex>
struct Tile {
	//position
	int tileX, tileY;
	float variance = 0.0f;
	//weight
	float weight = 1.0f;
	//the number of sample
	int currentSPP = 0;
	int targetSPP = 1;
};

struct VPL {
	ShadingData shadingData;
	Colour Le;
	bool fromLight = false;
	bool envornot = false; //from environment light or not
	Vec3 direnv;               //if from environmeny light, memory the direction
};
std::vector<VPL> vplList;
float lightPdf = 1.0f;
Colour lightIntensity = Colour(1.0f, 1.0f, 1.0f);
class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	Texture* albedo;
	std::thread** threads;
	Light* light;
	int numProcs;
	int MAX_DEPTH = 4;
	EnvironmentMap* envMap = nullptr;
	AreaLight* areaLight = nullptr;
	std::vector<VPL> vpls;
	const size_t MAX_VPLS = 120;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{


		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		scene->EnvironmentMap = dynamic_cast<EnvironmentMap*>(scene->background);
		envMap = dynamic_cast<EnvironmentMap*>(scene->EnvironmentMap);  
		for (Light* l : scene->lights) {
			areaLight = dynamic_cast<AreaLight*>(l);
			if (areaLight) {
			
				break;
			}
		}

		clear();


	}
	void clear()
	{
		film->clear();
	}
	/*Colour computeDirectFromVPLs(const ShadingData& shadingData, const std::vector<VPL>& vplList, Scene* scene)
	{
		

		Colour result(0.0f, 0.0f, 0.0f);
		for (const VPL& vpl : vplList)
		{
			Vec3 wi = (vpl.position - shadingData.x).normalize();
			Vec3 d = vpl.position - shadingData.x;
			float distance2 = d.x * d.x + d.y * d.y + d.z * d.z;
			float distance = std::sqrt(distance2);

			float cosTheta = Dot(shadingData.sNormal, wi);
			cosTheta = (cosTheta > 0.0f) ? cosTheta : 0.0f;

			float vplCos = Dot(vpl.normal, -wi);
			vplCos = (vplCos > 0.0f) ? vplCos : 0.0f;

			float G = (cosTheta * vplCos) / distance2;

			Ray shadowRay(shadingData.x + EPSILON * wi, wi);
			IntersectionData shadowHit = scene->traverse(shadowRay);
			if (shadowHit.t >= FLT_MAX || shadowHit.t > distance - EPSILON)
			{
				Colour f = shadingData.bsdf->evaluate(shadingData, wi);
				result += f * vpl.flux * G;
			}


		}

		return result;
	}*/

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{

		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);


	}


	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			Colour bsdf;
			float pdf;
			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::cosineHemispherePDF(wi);
			wi = shadingData.frame.toWorld(wi);
			bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour sampleBackgroundMIS(const ShadingData& shadingData, Sampler* sampler, Colour pathThroughput)
	{
		if (!envMap && !areaLight) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Vec3 dir_env;
		Colour L_env(0.0f, 0.0f, 0.0f);
		float p_env = 0.0f;

		if (envMap) {
			dir_env = envMap->sample(shadingData, sampler, L_env, p_env);
		}


		Vec3 dir_area;
		Colour L_area(0.0f, 0.0f, 0.0f);
		float p_area = 0.0f;

		if (areaLight && areaLight->triangle) {
			Vec3 localDir = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			Frame frame;
			frame.fromVector(shadingData.sNormal);
			dir_area = frame.toWorld(localDir);
			L_area = areaLight->emission;
			p_area = SamplingDistributions::cosineHemispherePDF(localDir);
		}

		// ----------------------------
		// 3. MIS 权重计算（Balance Heuristic）
		// ----------------------------
		float weight_env = (p_env > 0.0f && p_area > 0.0f) ? p_env / (p_env + p_area) : 1.0f;
		float weight_area = (p_env > 0.0f && p_area > 0.0f) ? p_area / (p_env + p_area) : 1.0f;

		// ----------------------------
		// 4. 合并贡献值
		// ----------------------------
		Colour contrib_env = (p_env > 0.0f) ? (L_env / (p_env + 1e-6f)) * weight_env : Colour(0.0f, 0.0f, 0.0f);
		Colour contrib_area = (p_area > 0.0f) ? (L_area / (p_area + 1e-6f)) * weight_area : Colour(0.0f, 0.0f, 0.0f);

		Colour totalContrib = contrib_env + contrib_area;

		return pathThroughput * totalContrib;
	}

	Colour pathTraceMIS(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			// 命中了物体
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0);
				}
			}

			// 直接光照部分
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			if (depth > MAX_DEPTH)
				return direct;

			// Russian Roulette
			float rrProb = (pathThroughput.Lum() < 0.9f) ? pathThroughput.Lum() : 0.9f;

			if (sampler->next() < rrProb)
			{
				pathThroughput /= rrProb;
			}
			else
			{
				return direct;
			}

			// 路径追踪一跳
			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			float pdf = SamplingDistributions::cosineHemispherePDF(wi);
			wi = shadingData.frame.toWorld(wi);
			Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			float cosTheta = Dot(wi, shadingData.sNormal);
			if (cosTheta < 0.0f) cosTheta = 0.0f;

			Colour scale = bsdf * (cosTheta / pdf);
			pathThroughput = pathThroughput * scale;


			r.init(shadingData.x + (wi * EPSILON), wi);
			return direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());
		}
		else
		{
			ShadingData envShadingData;
			envShadingData.sNormal = -r.dir;
			envShadingData.frame.fromVector(envShadingData.sNormal);
			envShadingData.bsdf = shadingData.bsdf;  // 传递当前 BSDF

			Colour envContrib = sampleBackgroundMIS(envShadingData, sampler, pathThroughput);
			return envContrib;
		}
	}


	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void connectToCamera(const Vec3& pos, const Vec3& normal, const Colour& contrib) {

		if (!scene->visible(pos, scene->camera.origin)) return;
		float px, py;
		if (!scene->camera.projectOntoCamera(pos, px, py)) return;

		Vec3 toCamera = (scene->camera.origin - pos);
		float dist2 = toCamera.lengthSq();
		if (dist2 <= 0.0f) return;

		toCamera = toCamera.normalize();
		float cosThetaLight = Dot(normal, toCamera);
		float cosThetaCamera = -Dot(scene->camera.viewDirection, toCamera);
		if (cosThetaLight <= 0.0f || cosThetaCamera <= 0.0f) return;

		float geom = (cosThetaLight * cosThetaCamera) / dist2;

		float cos4 = powf(cosThetaCamera, 4.0f);
		float we = 1.0f / (scene->camera.filmArea * cos4);

		int x = int(px);
		int y = int(py);
		if (x < 0 || x >= film->width || y < 0 || y >= film->height) return;
		film->splat(px, py, contrib * geom * we);
	}


	void lightTrace(Sampler* sampler) {
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (!light || pmf <= 0.0f) return;

		if (light->envornot()) {
			float pdfDir;
			Vec3 dir = light->sampleDirectionFromLight(sampler, pdfDir);
			if (pdfDir <= 0.0f) return;

			Colour Le = light->Emitted(dir);
			Colour contribution = Le / (pmf * pdfDir);

			Vec3 origin = scene->camera.origin - dir * 1e6f; 
			Ray r(origin + dir * EPSILON, dir);

			lightTracePath(r, Colour(1.0f,1.0f,1.0f), contribution, sampler, 0);
		}

		if (!light->envornot()) {
			float pdfPos, pdfDir;
			Vec3 pos = light->samplePositionFromLight(sampler, pdfPos);
			Vec3 dir = light->sampleDirectionFromLight(sampler, pdfDir);
			if (pdfPos <= 0.0f || pdfDir <= 0.0f) return;

			Colour Le = light->evaluate(-dir);
			Colour contribution = Le / (pmf * pdfPos * pdfDir);

			connectToCamera(pos, light->normal(dir), contribution);

			Ray r(pos + dir * EPSILON, dir);
			lightTracePath(r, Colour(1.0f,1.0f,1.0f), Le * Dot(dir, light->normal(pos)) / (pmf * pdfPos * pdfDir), sampler, 0);
		}
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		if (depth > MAX_DEPTH) return;

		IntersectionData isect = scene->traverse(r);
		if (isect.t == FLT_MAX) return;

		ShadingData shading = scene->calculateShadingData(isect, r);

		Vec3 toCamera = (scene->camera.origin - shading.x).normalize();
		Colour f = shading.bsdf->evaluate(shading, toCamera);
		if (!(f.r == 0.0f && f.g == 0.0f && f.b == 0.0f)) {
			Colour contrib = pathThroughput * f * Le;
			connectToCamera(shading.x, shading.sNormal, contrib);
		}

		if (depth > 3) {
			float prob = (pathThroughput.Lum() < 0.9f) ? pathThroughput.Lum() : 0.9f;
			if (sampler->next() > prob) return;
			pathThroughput /= prob;
		}

		float pdf;
		Colour bsdfVal;
		Vec3 nextDir = shading.bsdf->sample(shading, sampler, bsdfVal, pdf);
		if (pdf <= 0.0f) return;

		float cosTheta = Dot(nextDir, shading.sNormal);
		if (cosTheta <= 0.0f) return;

		Colour delta = bsdfVal * (cosTheta / pdf);
		pathThroughput = pathThroughput * delta;


		Ray nextRay(shading.x + nextDir * EPSILON, nextDir);
		lightTracePath(nextRay, pathThroughput, Le, sampler, depth + 1);
	}


	void render() {
		oidn::DeviceRef device = oidn::newDevice();
		device.set("setAffinity", true); 
		device.commit();


		film->initBuffers(device);
		film->incrementSPP();

		const int tileSize = 16;
		const int numTileX = (film->width + tileSize - 1) / tileSize;
		const int numTileY = (film->height + tileSize - 1) / tileSize;
		const int totalTiles = numTileX * numTileY;

		auto renderTile = [this, tileSize, numTileX, numTileY](int tileIndex, int threadIdx) {
			const int tileY = tileIndex / numTileX;
			const int tileX = tileIndex % numTileX;

			const unsigned int xStart = tileX * tileSize;
			const unsigned int xEnd = min(xStart + tileSize, film->width);
			const unsigned int yStart = tileY * tileSize;
			const unsigned int yEnd = min(yStart + tileSize, film->height);

			MTRandom* sampler = &samplers[threadIdx];

			for (unsigned int y = yStart; y < yEnd; ++y) {
				for (unsigned int x = xStart; x < xEnd; ++x) {
					const float px = x + 0.5f;
					const float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					Colour colourthought = Colour(1.f, 1.f, 1.f);
					Colour col = pathTrace(ray, colourthought, 0, sampler);

					film->splat(px, py, col);



				}
			}
			};

		// Create and launch threads
		for (int i = 0; i < numProcs; ++i) {
			threads[i] = new std::thread([=]() {
				for (unsigned int tileIndex = i; tileIndex < totalTiles; tileIndex += numProcs) {
					renderTile(tileIndex, i);
				}
				});
		}

		// Wait for threads to complete
		for (int i = 0; i < numProcs; ++i) {
			if (threads[i] && threads[i]->joinable()) {
				threads[i]->join();
				delete threads[i];
				threads[i] = nullptr;
			}
		}

		film->prepareDenoiserInput();

		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", film->colorBuffer,
			oidn::Format::Float3, film->width, film->height);
		filter.setImage("output", film->denoisedBuffer,
			oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.commit();
		auto start = std::chrono::high_resolution_clock::now();
		filter.execute();
		auto end = std::chrono::high_resolution_clock::now();
		std::cout << "OIDN: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
			<< "ms" << std::endl;
		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None)
			std::cerr << "OIDN Error: " << errorMessage << std::endl;

		std::vector<float> displayBuffer(film->width * film->height * 3);
		film->denoisedBuffer.read(
			static_cast<size_t>(0),  // 偏移量（size_t类型）
			static_cast<size_t>(film->width * film->height * 3 * sizeof(float)), // 字节数
			static_cast<void*>(displayBuffer.data())  // 目标指针（void*类型）
		);

		for (int i = 0; i < film->width * film->height; ++i) {

			float r = displayBuffer[i * 3 + 0];
			float g = displayBuffer[i * 3 + 1];
			float b = displayBuffer[i * 3 + 2];

			film->film[i] = Colour(r, g, b);
		}

		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				unsigned char rc, gc, bc;
				film->tonemap(x, y, rc, gc, bc);
				canvas->draw(x, y, rc, gc, bc);
			}
		}
		/*for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				int idx = (y * film->width + x) * 3;
				float r = displayBuffer[idx];
				float g = displayBuffer[idx + 1];
				float b = displayBuffer[idx + 2];
				unsigned char rc, gc, bc;

				film->tonemap(x, y, rc, gc, bc);



				canvas->draw(x, y, rc, gc, bc);
			}
		}*/


	}
	

	void renderpathtrace() {



		film->incrementSPP();


		const int tileSize = 16;
		const int numTileX = (film->width + tileSize - 1) / tileSize;
		const int numTileY = (film->height + tileSize - 1) / tileSize;
		const int totalTiles = numTileX * numTileY;
		std::vector<Tile> tiles(totalTiles);
		for (int i = 0; i < totalTiles; ++i) {
			tiles[i].tileX = i % numTileX;
			tiles[i].tileY = i / numTileX;
			tiles[i].targetSPP = 1;
		}
		auto renderTile = [this, tileSize, numTileX, numTileY](int tileIndex, int threadIdx) {
			const int tileY = tileIndex / numTileX;
			const int tileX = tileIndex % numTileX;

			const unsigned int xStart = tileX * tileSize;
			const unsigned int xEnd = min(xStart + tileSize, film->width);
			const unsigned int yStart = tileY * tileSize;
			const unsigned int yEnd = min(yStart + tileSize, film->height);

			MTRandom* sampler = &samplers[threadIdx];

			for (unsigned int y = yStart; y < yEnd; ++y) {
				for (unsigned int x = xStart; x < xEnd; ++x) {
					const float px = x + 0.5f;
					const float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					Colour colourthought = Colour(1.f, 1.f, 1.f);
					Colour col = pathTraceMIS(ray, colourthought, 0, sampler);
					//Colour col = pathTrace(ray, colourthought, 0, sampler);
					film->splat(px, py, col);

					unsigned char r = (unsigned char)(col.r * 255);
					unsigned char g = (unsigned char)(col.g * 255);
					unsigned char b = (unsigned char)(col.b * 255);
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
			};

		// Create and launch threads
		for (int i = 0; i < numProcs; ++i) {
			threads[i] = new std::thread([=]() {
				for (unsigned int tileIndex = i; tileIndex < totalTiles; tileIndex += numProcs) {
					renderTile(tileIndex, i);
				}
				});
		}
		
		// Wait for threads to complete
		for (int i = 0; i < numProcs; ++i) {
			if (threads[i] && threads[i]->joinable()) {
				threads[i]->join();
				delete threads[i];
				threads[i] = nullptr;
			}
		}
		updateSPP(tiles, tileSize);
	}


	void render2() {
		film->incrementSPP();

		const int tileSize = 32;
		const int numTileX = (film->width + tileSize - 1) / tileSize;
		const int numTileY = (film->height + tileSize - 1) / tileSize;
		const int totalTiles = numTileX * numTileY;

		std::vector<Tile> tiles(totalTiles);
		for (int i = 0; i < totalTiles; ++i) {
			tiles[i].tileX = i % numTileX;
			tiles[i].tileY = i / numTileX;
			tiles[i].targetSPP = 1;
		}

		std::vector<std::thread> threads(numProcs);

		for (int i = 0; i < numProcs; ++i) {
			threads[i] = std::thread([=]() {
				for (int tileIdx = i; tileIdx < totalTiles; tileIdx += numProcs) {
					MTRandom* sampler = &samplers[i];

					// 每个 tile 发射一定数量路径（可调）
					int numPaths = tileSize * tileSize;
					for (int j = 0; j < numPaths; ++j) {
						
						lightTrace(sampler);  
					}
				}
				});
		}

		// 等待所有线程结束
		for (int i = 0; i < numProcs; ++i) {
			if (threads[i].joinable()) threads[i].join();
		}

		// === 可选：自适应 SPP 更新 === //
		updateSPP(tiles, tileSize);

		// === Tonemap + Draw === //
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	Colour evaluateVPLContribution(const ShadingData& shadingData, const VPL& vpl, Scene* scene) {
		Vec3 toVPL = vpl.shadingData.x - shadingData.x;
		float dist2 = Dot(toVPL, toVPL);
		if (dist2 < 1e-9f) return Colour(0.f,0.0f,0.0f);

		Vec3 wi = toVPL.normalize();

		float dot1 = Dot(wi, shadingData.sNormal);
		float dot2 = Dot(wi, vpl.shadingData.sNormal);

		float cosThetaShading = (dot1 > 0.0f) ? dot1 : 0.0f;
		float cosThetaVPL = (-dot2 > 0.0f) ? -dot2 : 0.0f;

		if (cosThetaShading < 0.1f || cosThetaVPL < 0.1f) return Colour(0.f, 0.0f, 0.0f);

		float G = (cosThetaShading * cosThetaVPL) / (dist2 + EPSILON);
		if (G < 1e-4f) 
			return Colour(0.f, 0.0f, 0.0f);

		Ray shadowRay(shadingData.x + wi * EPSILON, wi);
		IntersectionData shadowHit = scene->traverse(shadowRay);
		float distToVPL = sqrtf(dist2);
		if (shadowHit.t < distToVPL - EPSILON) 
			return Colour(0.f, 0.0f, 0.0f);
		if (vpl.fromLight) {
			return vpl.Le * shadingData.bsdf->evaluate(shadingData, wi) * G;
		}
		else {
			Colour f_vpl = vpl.shadingData.bsdf->evaluate(vpl.shadingData, -wi);
			Colour f_surf = shadingData.bsdf->evaluate(shadingData, wi);
			return vpl.Le * f_vpl * f_surf * G;
		}
	}

	Colour evaluateEnvVPLContribution(const ShadingData& shadingData, const VPL& vpl, Scene* scene) {
		Vec3 wi = vpl.direnv;
		float cosTheta = Dot(shadingData.sNormal, wi);
		if (cosTheta < 1e-4f) return Colour(0.0f, 0.0f, 0.0f);
		Ray shadowRay(shadingData.x + wi * EPSILON, wi);
		IntersectionData shadowHit = scene->traverse(shadowRay);
		if (shadowHit.t < FLT_MAX) return Colour(0.0f, 0.0f, 0.0f);
		Colour f = shadingData.bsdf->evaluate(shadingData, wi);
		return vpl.Le * f * cosTheta;
	}


	Colour computeDirectFromVPLs(const ShadingData& shadingData, const std::vector<VPL>& vplList, Scene* scene) {
		Colour result(0.0f, 0.0f, 0.0f);

		for (const VPL& vpl : vplList) {
			if (vpl.envornot) {
				result += evaluateEnvVPLContribution(shadingData, vpl, scene);

			}
			if(!vpl.envornot) {
				result += evaluateVPLContribution(shadingData, vpl, scene);

			}
		}

		return result;
	}
	
	
	inline bool isSimilarFlux(const Colour& a, const Colour& b, float threshold = 0.01f) {
		float dr = fabsf(a.r - b.r);
		float dg = fabsf(a.g - b.g);
		float db = fabsf(a.b - b.b);
		return (dr + dg + db) < threshold;
	}
	inline bool canAddVPL() {
		return vpls.size() < MAX_VPLS;
	}

	inline void addVPL(const ShadingData& shadingData, const Colour& Le, bool fromLight, bool isEnvironment, const Vec3& envDir = Vec3(0.0f,0.0f,0.0f)) {
		if (!canAddVPL()) return;
		float lum = 0.2126f * Le.r + 0.7152f * Le.g + 0.0722f * Le.b;
		if (lum < 1e-4f || lum > 100.0f) return;

		VPL vpl;
		vpl.shadingData = shadingData;
		vpl.Le = Le;
		vpl.fromLight = fromLight;
		vpl.envornot = isEnvironment;
		vpl.direnv = envDir;
		vpls.push_back(vpl);
	}

	void VPLTracePath(Ray r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		if (!canAddVPL() || depth > MAX_DEPTH) return;

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t == FLT_MAX || shadingData.bsdf->isPureSpecular()) {
			return;
		}

		addVPL(shadingData, pathThroughput * Le, false, false);

		float pdfBSDF = 0.0f;
		Colour bsdfVal;
		Vec3 newDir = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdfBSDF);
		float cosine = fabsf(Dot(newDir, shadingData.sNormal));
		if (pdfBSDF <= 1e-6f) return;
		pathThroughput = pathThroughput * bsdfVal * cosine / pdfBSDF;

		float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() >= russianRouletteProbability) return;

		pathThroughput /= russianRouletteProbability;

		r.init(shadingData.x + newDir * EPSILON, newDir);
		VPLTracePath(r, pathThroughput, Le, sampler, depth + 1);
	}

	void generateEnvironmentVPL(Light* light, float pmf, int N_VPLs, Sampler* sampler) {
		float pdfDir;
		Vec3 envDir = light->sampleDirectionFromLight(sampler, pdfDir);
		if (pdfDir <= 0.0f) return;

		Colour envEmit = light->Emitted(envDir);
		Colour LE = envEmit / (pmf * pdfDir * float(N_VPLs));

		ShadingData shadingData(Vec3(0.f,0.0f,0.0f), Vec3(0.f,0.0f,0.0f));
		addVPL(shadingData, LE, true, true, envDir);
	}

	void generateSurfaceVPL(Light* light, float pmf, int N_VPLs, Sampler* sampler) {
		float pdfPosition;
		Vec3 pos = light->samplePositionFromLight(sampler, pdfPosition);
		if (pdfPosition <= 0.0f) return;

		ShadingData shadingData(pos, light->normal(pos));
		Colour emitted = light->Emitted(pos);
		Colour LE = emitted / (pmf * pdfPosition * float(N_VPLs));
		addVPL(shadingData, LE, true, false);

		float pdfDir;
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDir);
		if (pdfDir <= 0.0f) return;

		float cosLight = max(0.0f, Dot(wi, light->normal(pos)));
		Colour le = emitted * cosLight / (pmf * pdfPosition * pdfDir * float(N_VPLs));
		Ray r(shadingData.x + wi * EPSILON, wi);
		VPLTracePath(r, Colour(1.f,1.f,1.f), le, sampler,0);
	}

	void traceVPLs(Sampler* sampler, int N_VPLs) {
		vpls.clear();
		vpls.reserve(MAX_VPLS);

		for (int i = 0; i < N_VPLs; ++i) {
			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);
			if (!light || pmf <= 0.0f) continue;

			if (light->envornot()) {
				generateEnvironmentVPL(light, pmf, N_VPLs, sampler);
			}
			else {
				generateSurfaceVPL(light, pmf, N_VPLs, sampler);
			}
		}
	}

	void renderVPLS() {
		film->incrementSPP();

		const int tileSize = 16;
		const int numTileX = (film->width + tileSize - 1) / tileSize;
		const int numTileY = (film->height + tileSize - 1) / tileSize;
		const int totalTiles = numTileX * numTileY;

		std::vector<Tile> tiles(totalTiles);

		for (int i = 0; i < totalTiles; ++i) {
			tiles[i].tileX = i % numTileX;
			tiles[i].tileY = i / numTileX;
			tiles[i].targetSPP = 1;
		}

		traceVPLs(samplers, 16);

		auto renderTile = [&](int tileIndex, int threadIdx) {
			const int tileY = tileIndex / numTileX;
			const int tileX = tileIndex % numTileX;

			const unsigned int xStart = tileX * tileSize;
			const unsigned int yStart = tileY * tileSize;
			const unsigned int xEnd = min(xStart + tileSize, film->width);
			const unsigned int yEnd = min(yStart + tileSize, film->height);

			MTRandom* sampler = &samplers[threadIdx];

			Tile& tile = tiles[tileIndex];

			for (int s = 0; s < tile.targetSPP; ++s) {
				for (int y = yStart; y < yEnd; ++y) {
					for (int x = xStart; x < xEnd; ++x) {
						float px = x + sampler->next();
						float py = y + sampler->next();

						Ray ray = scene->camera.generateRay(px, py);
						IntersectionData intersection = scene->traverse(ray);
						ShadingData shadingData = scene->calculateShadingData(intersection, ray);

						Colour col;
						if (shadingData.bsdf && shadingData.bsdf->isLight()) {
							col = shadingData.bsdf->emission;
						}
						else if (shadingData.t < FLT_MAX) {
							col = computeDirectFromVPLs(shadingData, vpls, scene);
						}
						else {
							col = scene->background->evaluate(ray.dir);
						}
						film->splat(px, py, col);
					}
				}
			}
			tile.currentSPP += tile.targetSPP;
			};

		for (int i = 0; i < numProcs; ++i) {
			threads[i] = new std::thread([&, i]() {
				for (int tileIndex = i; tileIndex < totalTiles; tileIndex += numProcs)
					renderTile(tileIndex, i);
				});
		}

		for (int i = 0; i < numProcs; ++i) {
			threads[i]->join();
			delete threads[i];
			threads[i] = nullptr;
		}

		updateSPP(tiles, tileSize);

		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}


	void updateSPP(std::vector<Tile>& tiles, int tileSize) {
		if (getSPP() % 4 != 0) return;

		float totalVar = 0.f;
		for (auto& t : tiles) {
			float v = 0.f;
			int count = 0;
			auto sample = [&](int x, int y) {
				x = (x < 0) ? 0 : (x >= film->width ? film->width - 1 : x);
				y = (y < 0) ? 0 : (y >= film->height ? film->height - 1 : y);
				int idx = y * film->width + x;
				int spp = max(1, film->sppBuffer[idx]);
				return film->film[idx] * (1.f / spp);
				};

			for (int y = 0; y < tileSize; ++y)
				for (int x = 0; x < tileSize; ++x) {
					Colour c = sample(t.tileX * tileSize + x, t.tileY * tileSize + y);
					float l0 = c.Lum(), l1 = 0.f, w = 0.f;
					for (int dy = -1; dy <= 1; ++dy)
						for (int dx = -1; dx <= 1; ++dx) {
							float wf = 1.f;
							l1 += sample(t.tileX * tileSize + x + dx, t.tileY * tileSize + y + dy).Lum() * wf;
							w += wf;
						}
					l1 /= w;
					v += (l0 - l1) * (l0 - l1);
					++count;
				}

			t.variance = (count > 0) ? v / count : 0.f;
			totalVar += t.variance;
		}

		for (auto& t : tiles) {
			t.weight = (totalVar > 0.f) ? t.variance / totalVar : 1.f / tiles.size();
			int total = film->width * film->height;
			int tilePixels = tileSize * tileSize;
			t.targetSPP = max(1, int((t.weight * total) / tilePixels));
		}
	}

	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};