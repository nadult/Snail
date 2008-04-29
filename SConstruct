import os

includesSDL = '-I/usr/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT' #os.popen('sdl-config --cflags').read() 
includes = includesSDL + ' -I /home/someone/veclib -I /home/someone/baselib '

libsPaths = [ '/usr/local/lib', '/home/someone/baselib/', '/usr/X11R6/lib' ]
libs = [ 'pthread', 'SDL', 'baselib' ]

release = Environment (
		CXX = '/usr/local/gcc-4.3/bin/g++ -fopenmp',
		CXXFLAGS = includes + '-O3 -msse3 -ffast-math -mfpmath=sse -funroll-loops -fno-rtti -DNDEBUG -g',
		CPPPATH = '.',
		LIBPATH = libsPaths
	)
debug = Environment (
		CXX= '/usr/local/gcc-4.3/bin/g++',
		CXXFLAGS = includes + '-O0 -g',
		CPPPATH = '.',
		LIBPATH = libsPaths
	)

env=release
if int(ARGUMENTS.get('debug',0)):
	env=debug

def BuildObject(file,dir):
	return env.Object('build/'+dir+file[0:-4] , dir+file)

def BuildObjects(files,dir):
	fileList = Split(files)
	outList = []
	for file in fileList:
		obj = BuildObject(file,dir)
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

baseObjects = BuildObjects( ListCppFiles('./'), './')

env.Program('rtracer', baseObjects, LIBS=libs )
