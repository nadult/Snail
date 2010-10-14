#include "comm.h"


namespace comm {

	PSocket operator<<(PSocket sock, const LoadNewModel &lm) {
		int strSize = lm.name.size();
		sock	<< strSize << Data((void*)&lm.name[0], strSize)
				<< lm.resx << lm.resy << lm.nNodes << lm.rebuild << lm.flipNormals << lm.swapYZ;
	}

	PSocket operator>>(PSocket sock, LoadNewModel &lm) {
		int strSize;
		sock >> strSize;
		lm.name.resize(strSize);
		sock >> Data(&lm.name[0], strSize) >> lm.resx >> lm.resy >> lm.nNodes >>
				lm.rebuild >> lm.flipNormals >> lm.swapYZ;
	}

}

