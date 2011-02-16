#include "shading/material.h"
#include "shading/simple_material.h"
#include "shading/tex_material.h"
#include "shading/uber_material.h"
#include "shading/transparent_material.h"
#include <sstream>
#include <fstream>

namespace shading {
	using namespace sampling;

	Material::~Material() { }

	Material *NewMaterial(PTexture tex, bool NDotL) {
		PSampler sampler = sampling::NewSampler(tex);

		return sampler?
					NDotL?	(Material*)new MaterialWrapper< TexMaterial<1> >(sampler):
							(Material*)new MaterialWrapper< TexMaterial<0> >(sampler)
				:	NDotL?	(Material*)new MaterialWrapper< SimpleMaterial<1> >(Vec3f(1.0f, 1.0f, 1.0f)):
							(Material*)new MaterialWrapper< SimpleMaterial<0> >(Vec3f(1.0f, 1.0f, 1.0f));
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

	MaterialDesc::MaterialDesc()
		:ambient(0, 0, 0), diffuse(1, 1, 1), specular(0, 0, 0), emissive(0, 0, 0),
		 transmission(0, 0, 0), illuminationModel(0), dissolveFactor(0),
		 specularExponent(0), refractionIndex(0) { } 

	const vector<MaterialDesc> LoadMaterialDescs(const string &fileName) {
		std::filebuf fb;
		if(!fb.open (fileName.c_str(),std::ios::in)) {
			std::cout << "Error while opening: " << fileName << '\n';
			return vector<MaterialDesc>();
		}
		std::istream is(&fb);

		vector<MaterialDesc> mats;
		MaterialDesc newMat;

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
					newMat = MaterialDesc();
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
				newMat.dissolveFactor = atof(token.c_str());
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

		return mats;
	}

	static void LoadTex(const string &name, const string &path, TexDict &dict) {
		if(name != "" && dict.find(name) == dict.end()) {
			try {
				PTexture tex = new gfxlib::Texture;
				Loader(path + name) & *tex;
				dict[name] = tex;
			}
			catch(const Exception &ex) {
				std::cout << ex.what() << '\n';
			}
		}
	}

	const TexDict LoadTextures(const vector<MaterialDesc> &matDescs, const string &texPath) {
		TexDict texDict;

		for(size_t n = 0; n < matDescs.size(); n++) {
			const MaterialDesc &mat = matDescs[n];
			LoadTex(mat.dissolveMap, texPath, texDict);
			LoadTex(mat.diffuseMap, texPath, texDict);
			LoadTex(mat.dissolveMap, texPath, texDict);
			LoadTex(mat.ambientMap, texPath, texDict);
			LoadTex(mat.specularMap, texPath, texDict);
			LoadTex(mat.emissiveMap, texPath, texDict);
			LoadTex(mat.exponentMap, texPath, texDict);
		}

		return texDict;
	}

	const MatDict MakeMaterials(const vector<MaterialDesc> &matDescs, const TexDict &texDict) {
		std::map<string, PMaterial> out;

		for(size_t n = 0; n < matDescs.size(); n++) {
			const MaterialDesc &mat = matDescs[n];
			PMaterial newMat;
			try {
				TexDict::const_iterator it1 = texDict.find(mat.diffuseMap);
				TexDict::const_iterator it2 = texDict.find(mat.dissolveMap);
				PTexture diff  = it1 == texDict.end()? (PTexture)0 : it1->second;
				PTexture trans = it2 == texDict.end()? (PTexture)0 : it2->second;

				if(diff && trans)
					newMat = new MaterialWrapper<TransparentMaterial<1>>(
								sampling::NewSampler(diff), sampling::NewSampler(trans));
				else {
					newMat = diff?NewMaterial(diff) : new MaterialWrapper<UberMaterial>(mat);
				}
			}
			catch(const Exception &ex) {
				std::cout << ex.what() << '\n';
				newMat = NewMaterial(0);
			}

			out[mat.name] = newMat;
		}

		return out;
	}

}
