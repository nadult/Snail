import os


libs = [ 'baselib', 'glfw', 'pthread', 'png', 'gfxlib' ] 
libsLinux = [ 'GL', 'GLU', 'Xrandr' ]
libsWin32 = [ 'opengl32', 'glu32' ]

if Environment()["PLATFORM"] == 'posix': libs += libsLinux
else: libs += libsWin32

default = Environment (
		ENV = os.environ,
		PLATFORM = 'posix',
		CXX = 'ccache /usr/local/gcc-4.3.2/bin/g++ -std=c++0x',
		CPPPATH = '.'
	)

if int(ARGUMENTS.get('-m32',0)):
	default=default.Clone( CXX='g++ -m32')

release = default.Clone(
	CXXFLAGS='-O3 -mtune=core2 -mssse3 -mfpmath=sse -fvariable-expansion-in-unroller -fprefetch-loop-arrays \
				-funroll-all-loops -fpeel-loops -g -DNDEBUG -Wstrict-aliasing=2 -Wno-unused -Wno-conversion',
	BUILDDIR='build/release/'
)
debug = default.Clone(
	CXXFLAGS='-O0 -msse2 -g -gdwarf-2',
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
	formatsObjects = BuildObjects( env, ListCppFiles('formats/'), 'formats/')
	bihObjects = BuildObjects( env, ListCppFiles('bih/'), 'bih/')
	env.Program(progName, baseObjects+formatsObjects+bihObjects, LIBS=libs )

Build( release, 'rtracer' )
Build( debug, 'rtracerd' )

