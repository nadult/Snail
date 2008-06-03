#include "camera.h"

void CameraConfigs::Serialize(Serializer &sr) {
	std::map<string,Camera>::iterator it=data.begin();
	int count=data.size();
	sr&count;

	if(sr.IsLoading()) {
		for(int n=0;n<count;n++) {
			string str; Camera cam;
			sr & str & cam;
			data[str] = cam;
		}
	}
	else {
		while(it!=data.end()) {
			string tmp=it->first;
			sr & tmp & it->second;
			++it;
		}
	}
}

void CameraConfigs::AddConfig(const string &str,const Camera &cam) {
	data[str] = cam;
}

bool CameraConfigs::GetConfig(const string &str,Camera &cam) const {
	std::map<string,Camera>::const_iterator it=data.find(str);
	if(it!=data.end()) { cam=it->second; return 1; }
	return 0;
}
