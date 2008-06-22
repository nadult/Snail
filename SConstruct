import os


libs = [ 'pthread', 'baselib', 'glfw' ]
libsLinux = [ 'GL', 'Xrandr' ]
libsWin32 = [ 'opengl32' ]

if Environment()["PLATFORM"] == 'posix': libs += libsLinux
else: libs += libsWin32

default = Environment (
		ENV = os.environ,
		PLATFORM = 'posix',
		CXX = 'ccache g++',
		CPPPATH = '.'		
	)

if int(ARGUMENTS.get('32bit',0)):
	default=default.Clone( CXX='g++ -m32')

release = default.Clone(
	CXXFLAGS='-O3 -mssse3 -ffast-math -mfpmath=sse -funroll-loops -DNDEBUG -g',
	BUILDDIR='build/release/'
)
debug = default.Clone(
	CXXFLAGS='-O0 -msse2 -g',
	BUILDDIR='build/debug/'
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

def Build( env, progName ):
	baseObjects = BuildObjects( env, ExcludeFromList(ListCppFiles('./'),'gen_bihtrav.cpp'), './')
	env.Program(progName, baseObjects, LIBS=libs )

Build( release, 'rtracer' )
Build( debug, 'rtracerd' )

