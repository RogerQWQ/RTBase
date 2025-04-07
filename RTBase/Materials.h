#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosThetaI, float iorI, float iorT) {
		
		float etaI = iorI, etaT = iorT;
		float sinThetaI = sqrt(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
		float sinThetaT = etaI * sinThetaI / etaT;

		if (sinThetaT >= 1.0f) return 1.0f; 

		float cosThetaT = sqrt(std::max(0.0f, 1.0f - sinThetaT * sinThetaT));
		float Rs = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
		float Rp = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
		return 0.5f * (Rs * Rs + Rp * Rp);
	}
	static Colour fresnelConductor(float cosTheta, float ior, float k)
	{
		// Add code here
		cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
		float cosTheta2 = cosTheta * cosTheta;
		float sinTheta2 = 1.0f - cosTheta2;

		float a = ior * ior;
		float k_sq = k * k;
		float numerator = (a - 1.0f) * cosTheta2 + (a + k_sq - 1.0f) * sinTheta2;
		float denominator = (a + k_sq) * cosTheta2 + (a - k_sq) * sinTheta2;
		float F0 = std::max(0.0f, std::min(1.0f, numerator / denominator));

		return Colour(F0, F0, F0);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		float cosTheta = std::max(wi.z, 0.0f);
		if (cosTheta <= 0.0f) return 0.0f;

		float alpha2 = alpha * alpha;
		float tanTheta = std::sqrt(1.0f - cosTheta * cosTheta) / cosTheta;
		float denom = alpha2 * tanTheta * tanTheta + 1.0f;

		return (std::sqrt(denom) - 1.0f) / (2.0f * denom);
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		float lambdaWi = lambdaGGX(wi, alpha);
		float lambdaWo = lambdaGGX(wo, alpha);
		return 2.0f / (lambdaWi + lambdaWo + 1e-7f);
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		float cosTheta = std::max(h.z, 0.0f); 
		float alpha2 = alpha * alpha;
		float denom = (cosTheta * cosTheta) * (alpha2 - 1) + 1;
		return (alpha2) / (M_PI * denom * denom);
	}
};

class BSDF
{
public:
	Colour emission;
	
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	

	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
	


};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wi = Vec3(0, 1, 0);
		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float cosTheta = Dot(wi, shadingData.sNormal);
		if (cosTheta <= 0.0f) return Colour(0.f, 0.f, 0.f); 
		return albedo->sample(shadingData.tu, shadingData.tv) * (1.0f / M_PI);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		float cosTheta = Dot(wi, shadingData.sNormal);
		if (cosTheta <= 0.0f) return 0.0f;
		return cosTheta / M_PI;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};




class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	
};




class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;

	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		float cosThetaO = wo.z;
		float F = ShadingHelper::fresnelDielectric(cosThetaO, extIOR, intIOR);

		// Reflection
		if (sampler->next() < F)
		{
			Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
			reflectedColour = Colour(F, F, F);
			pdf = F;
			return shadingData.frame.toWorld(wr);
		}

		// Transmission
		float sqrtTerm = 1 - (intIOR / extIOR) * (intIOR / extIOR) * (1 - cosThetaO * cosThetaO);
		if (sqrtTerm <= 0)
		{
			pdf = 0;
			return Vec3(0, 0, 0);
		}
		float cosThetaT = sqrt(sqrtTerm);
		Vec3 wt = Vec3 (sin(shadingData.frame.toLocal(shadingData.wo).x) * cos(shadingData.frame.toLocal(shadingData.wo).y),
			sin(shadingData.frame.toLocal(shadingData.wo).x) * sin(shadingData.frame.toLocal(shadingData.wo).y),
			-cosThetaT);
		reflectedColour = Colour((1 - F) / M_PI, (1 - F) / M_PI, (1 - F) / M_PI);
		pdf = (1 - F);
		return shadingData.frame.toWorld(wt);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		float cosThetaO = wo.z;
		float cosThetaI = wiLocal.z;
		float F = ShadingHelper::fresnelDielectric(cosThetaO, extIOR, intIOR);

		// Reflection
		if (wiLocal.z > 0)
		{
			return albedo->sample(shadingData.tu, shadingData.tv) * F / abs(cosThetaI);
		}

		// Transmission
		float sqrtTerm = 1 - (intIOR / extIOR) * (intIOR / extIOR) * (1 - cosThetaO * cosThetaO);
		if (sqrtTerm <= 0)
		{
			return Colour(0, 0, 0);
		}
		float cosThetaT = sqrt(sqrtTerm);
		return albedo->sample(shadingData.tu, shadingData.tv) * (1 - F) / abs(cosThetaI);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0;
	}

	bool isPureSpecular()
	{
		return true;
	}

	bool isTwoSided()
	{
		return true; 
	}

	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	

};

//class GlassBSDF : public BSDF
//{
//public:
//	Texture* albedo;
//	float intIOR;
//	float extIOR;
//	GlassBSDF() = default;
//	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
//	{
//		albedo = _albedo;
//		intIOR = _intIOR;
//		extIOR = _extIOR;
//	}
//	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
//	{
//		// Replace this with Glass sampling code
//		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
//		pdf = wi.z / M_PI;
//		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
//		wi = shadingData.frame.toWorld(wi);
//		return wi;
//	}
//	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
//	{
//		// Replace this with Glass evaluation code
//		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
//	}
//	float PDF(const ShadingData& shadingData, const Vec3& wi)
//	{
//		// Replace this with GlassPDF
//		Vec3 wiLocal = shadingData.frame.toLocal(wi);
//		return SamplingDistributions::cosineHemispherePDF(wiLocal);
//	}
//	bool isPureSpecular()
//	{
//		return true;
//	}
//	bool isTwoSided()
//	{
//		return false;
//	}
//	float mask(const ShadingData& shadingData)
//	{
//		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
//	}
//};


class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	Texture* albedo;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR, Texture* _albedo)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
		albedo = _albedo;
	}
	
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
	
};