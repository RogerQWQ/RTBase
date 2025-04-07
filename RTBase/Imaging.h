#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"
#include<string>
#include <iostream> 
#include <OpenImageDenoise/oidn.hpp>
// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) < 0.5f && fabs(y) < 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};
class GaussianFilter : public ImageFilter {
private:
	float sigma; // 控制衰减速度（通常设为1.0）
public:
	GaussianFilter(float s = 1.0f) : sigma(s) {}

	float filter(float x, float y) const override {
		float r2 = x * x + y * y;
		return exp(-r2 / (2 * sigma * sigma)); // 二维高斯函数
	}

	int size() const override {
		return 2; // 影响范围半径（2表示5x5区域） 
	}
};
class Film
{
public:
	Colour* film;
	Colour* albedoBuffer;
	Colour* normalBuffer;
	unsigned int width;
	unsigned int height;
	int SPP;
	int* sppBuffer;
	
	oidn::BufferRef colorBuffer;
	oidn::BufferRef denoisedBuffer;
	ImageFilter* filter;
	void initBuffers(oidn::DeviceRef& device) {
		const size_t bufferSize = width * height * 3 * sizeof(float);
		colorBuffer = device.newBuffer(bufferSize);
		denoisedBuffer = device.newBuffer(bufferSize);
	}
	void splat(const float x, const float y, const Colour& L) {
		float filterWeights[25];
		unsigned int indices[25];
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(j, i); // 修改这里
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			int index = indices[i];
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
			sppBuffer[index]++;
		}
	}
	void splatAlbedo(int x, int y, const Colour& albedo) {
		albedoBuffer[y * width + x] = albedo;
	}

	void splatNormal(int x, int y, const Colour& normal) {
		normalBuffer[y * width + x] = normal;
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		float exposure = 1.f;
		Colour pixel = film[(y * width) + x] * exposure / (float)SPP;
		r = std::min(powf(std::max(pixel.r, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		g = std::min(powf(std::max(pixel.g, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		b = std::min(powf(std::max(pixel.b, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
	}
	void tonemapRGB(float inR, float inG, float inB,
		unsigned char& outR, unsigned char& outG, unsigned char& outB)
	{
		// 这里和原来的 Film::tonemap 逻辑差不多
		float exposure = 1.f;
		float rExp = std::pow(std::max(inR * exposure, 0.0f), 1.f / 2.2f) * 255.f;
		outR = (unsigned char)std::clamp(rExp, 0.f, 255.f);

		float gExp = std::pow(std::max(inG * exposure, 0.0f), 1.f / 2.2f) * 255.f;
		outG = (unsigned char)std::clamp(gExp, 0.f, 255.f);

		float bExp = std::pow(std::max(inB * exposure, 0.0f), 1.f / 2.2f) * 255.f;
		outB = (unsigned char)std::clamp(bExp, 0.f, 255.f);
	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		sppBuffer = new int[width * height];
		clear();
		filter = _filter;
		albedoBuffer = new Colour[width * height];   // ✅ 需要这句！
		normalBuffer = new Colour[width * height];
	}
	void prepareDenoiserInput() {
		float scale = 1.0f;
		std::vector<float> tempBuffer(width * height * 3);

		for (int i = 0; i < width * height; ++i) {
			Colour c = film[i] * scale;
			tempBuffer[i * 3 + 0] = c.r;
			tempBuffer[i * 3 + 1] = c.g;
			tempBuffer[i * 3 + 2] = c.b;
		}

		
		colorBuffer.write(
			0, // byteOffset
			tempBuffer.size() * sizeof(float), // byteSize
			reinterpret_cast<const void*>(tempBuffer.data()) // srcHostPtr
		);

	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		memset(sppBuffer, 0, width * height * sizeof(int));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
	void clearAlbedoAndNormal()
	{
		memset(albedoBuffer, 0, width * height * sizeof(Colour));
		memset(normalBuffer, 0, width * height * sizeof(Colour));
	}
};