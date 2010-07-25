#include "shading/material.h"
#include "shading/simple_material.h"
#include "shading/tex_material.h"
#include "shading/transparent_material.h"
#include <sstream>
#include <fstream>

namespace shading {
	using namespace sampling;

	Material::~Material() { }

	Material *NewMaterial(const string &texName, bool NDotL) {
		PSampler sampler = sampling::NewSampler(texName);

		return sampler?
					NDotL?	(Material*)new MaterialWrapper< TexMaterial<1> >(sampler):
							(Material*)new MaterialWrapper< TexMaterial<0> >(sampler)
				:	NDotL?	(Material*)new MaterialWrapper< SimpleMaterial<1> >(Vec3f(1.0f,1.0f,1.0f)):
							(Material*)new MaterialWrapper< SimpleMaterial<0> >(Vec3f(1.0f,1.0f,1.0f)) ;
	}

	static const Vec3f ReadColor(std::stringstream &ss) {
		string x, y, z;
		ss >> x;
		if(x == "spectral")
			ThrowException("spectral files not supported");
		if(x == "xyz")
			ThrowException("xyz values not supported");
		Vec3f out;
		out.x = atof(x.c_str());
		ss >> y >> z;
		if(y == "" && z == "")
			out.y = out.z = out.x;
		else {
			out.y = atof(y.c_str());
			out.z = atof(z.c_str());
		}
		return out;	
	}

	struct Mat {
		Vec3f ambient, diffuse, specular, emissive;
		Vec3f transmission;
		int illuminationModel;
		float dissolveFactor;
		float specularExponent;
		int refractionIndex;

		string ambientMap, diffuseMap;
		string specularMap, emissiveMap;
		string exponentMap, dissolveMap;

		string name;
	};

	const MatDict LoadMaterials(const string &fileName, const string &texPath) {
		std::filebuf fb;
		if(!fb.open (fileName.c_str(),std::ios::in)) {
			std::cout << "Error while opening: " << fileName << '\n';
			return MatDict();
		}

		std::istream is(&fb);
		std::map<string, PMaterial> out;

		vector<Mat> mats;
		Mat newMat;

		while(is.good()) {
			string line;
			getline(is, line);
			if(!is.good())
				break;

			for(int n = 0; n < line.size(); n++) if(line[n] == '#') {
				line.resize(n);
				break;
			}
			std::stringstream ss(line);

			string token; ss >> token;
			if(token == "newmtl") {
				if(newMat.name != "") {
					mats.push_back(newMat);
					newMat = Mat();
				}
				ss >> newMat.name;
			}
			else if(token[0] == 'K') {
					 if(token == "Ka") newMat.ambient = ReadColor(ss);
				else if(token == "Kd") newMat.diffuse = ReadColor(ss);
				else if(token == "Ks") newMat.specular = ReadColor(ss);
				else if(token == "Ke") newMat.emissive = ReadColor(ss);
			}
			else if(token == "Tf") newMat.transmission = ReadColor(ss);
			else if(token == "illum") ss >> newMat.illuminationModel;
			else if(token == "d") {
				string token;
				ss >> token;
				if(token == "-halo")
					ThrowException("-halo parameter not supported");
				newMat.dissolveFactor = atoi(token.c_str());
			}
			else if(token == "Ns") ss >> newMat.specularExponent;
			else if(token == "Ni") ss >> newMat.refractionIndex;
			else if(token[0] == 'm') {
				if(token == "map_Ka") ss >> newMat.ambientMap;
				if(token == "map_Kd") ss >> newMat.diffuseMap;
				if(token == "map_Ks") ss >> newMat.specularMap;
				if(token == "map_Ke") ss >> newMat.emissiveMap;
				if(token == "map_Ns") ss >> newMat.exponentMap;
				if(token == "map_d")  ss >> newMat.dissolveMap;
			}
			else if(token == "") continue;
			else {
			//	ThrowException("Unknown token: ", token);
			}
		}

		if(newMat.name != "")
			mats.push_back(newMat);

		for(size_t n = 0; n < mats.size(); n++) {
			const Mat &mat = mats[n];
			PMaterial newMat;
			try {
				if(mat.dissolveMap != "") {
					PSampler col = sampling::NewSampler(texPath + mat.diffuseMap);
					PSampler trans = sampling::NewSampler(texPath + mat.dissolveMap);

					newMat = new MaterialWrapper<TransparentMaterial<1>>(col, trans);
				}
				else
					newMat = NewMaterial(texPath + mat.diffuseMap, 1);
			}
			catch(const Exception &ex) {
				std::cout << ex.what() << '\n';
				newMat = NewMaterial("");
			}

			out[mat.name] = newMat;
		}

		return out;
	}

}
