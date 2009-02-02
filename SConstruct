import os

libs = [ 'baselib', 'glfw', 'gfxlib', 'png', 'pthread', 'z',
			'bulletdynamics','bulletcollision', 'bulletmath' ] 
libsLinux = libs + [ 'GL', 'GLU', 'Xrandr' ]
libsWin32 = libs + [ 'opengl32', 'glu32', 'kernel32', 'ws2_32' ]

envLinux32 = Environment (
		ENV = os.environ,
		PLATFORM = 'posix',
		TARGET_PLATFORM='linux32',
	#	CXX = '/usr/local/gcc-4.4/bin/g++ -O3 -std=gnu++0x -msse2 -m32 -finline-limit=200',
		CXX = '/usr/local/gcc-4.3.2/bin/g++ -O3 -std=gnu++0x -msse2 -m32',
		CPPPATH = '.',
	)
envLinux64 = Environment (
		ENV = os.environ,
		PLATFORM = 'posix',
		TARGET_PLATFORM='linux64',
	#	CXX = '/usr/local/gcc-4.4/bin/g++ -std=gnu++0x -msse2 -ffast-math',
		CXX = '/usr/local/gcc-4.3.2/bin/g++ -std=gnu++0x -msse2 -ffast-math',
		CPPPATH = '.',
	)
envWin32 = Environment (
		ENV = os.environ,
		PLATFORM = 'posix',
		TARGET_PLATFORM='win32',
		CXX = '/usr/local/mingw32-4.3/bin/i686-mingw32-g++ -std=gnu++0x -msse2 -ffast-math -finline-limit=200 -mthreads',
		CPPPATH = ['.', '/usr/local/mingw32-4.3/include'],
		LIBPATH = '/usr/local/mingw32-4.3/lib',
	)

def ReleaseEnv(env):
	return env.Clone(
		CXXFLAGS='-O3 -mfpmath=sse -g -DNDEBUG -Wstrict-aliasing=2 -Wno-unused -Wno-conversion',
		BUILDDIR='build/'+env['TARGET_PLATFORM']+'_release/'
	)
def DebugEnv(env):
	return env.Clone(
		CXXFLAGS='-O0 -msse2 -g -gdwarf-2',
		BUILDDIR='build/'+env['TARGET_PLATFORM']+'_debug/'
	)

def BuildObject(env,file,dir):
	buildDir = env['BUILDDIR']
	return env.Object(buildDir+dir+file[0:-4] , dir+file)

def BuildObjects(env,files,dir):
	fileList = Split(files)
	outList = []
	for file in fileList:
		obj = BuildObject(env,file,dir)
		outList.append(obj)
	return outList

def ListCppFiles(dir):
	outList = []
	files = os.listdir(dir)
	for filename in files:
		if filename.endswith('.cpp'):
			outList.append(filename)
	return outList

def ExcludeFromList(tList,tObj):
	outList = []
	for obj in tList:
		if obj != tObj:
			outList.append(obj)
	return outList

def Build( env, progName, libs ):
	baseObjects		= BuildObjects( env, ExcludeFromList(ListCppFiles('./'),'gen_bihtrav.cpp'), './')
	formatsObjects	= BuildObjects( env, ListCppFiles('formats/'	), 'formats/')
	bihObjects		= BuildObjects( env, ListCppFiles('bih/'		), 'bih/')
	shadingObjects	= BuildObjects( env, ListCppFiles('shading/'	), 'shading/')
	samplingObjects	= BuildObjects( env, ListCppFiles('sampling/'	), 'sampling/')
	gameObjects		= BuildObjects( env, ListCppFiles('game/'		), 'game/')
	env.Program(	progName,
					baseObjects+formatsObjects+bihObjects+samplingObjects+shadingObjects+gameObjects,
					LIBS=libs )

Build( ReleaseEnv(envLinux64), 'rtracer' , libsLinux )
Build( DebugEnv  (envLinux64), 'rtracerd', libsLinux )

Build( ReleaseEnv(envLinux32), 'rtracer32' , libsLinux )
Build( DebugEnv  (envLinux32), 'rtracer32d', libsLinux )

Build( ReleaseEnv(envWin32), 'rtracer.exe' , libsWin32 )
Build( DebugEnv  (envWin32), 'rtracerd.exe', libsWin32 )

