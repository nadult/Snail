icl /arch:SSE2 /Qopenmp /ML -I ../../libs/baselib/ -I ../../libs/veclib/ -I ../../libs/boost_1_35_0/ -I ../../libs/glfw/include *.cpp ../../libs/glfw/glfw.lib ../../libs/baselib/baselib.lib opengl32.lib user32.lib -o rtracer.exe /Ox /fp:fast=2 /arch:SSE2 /Qunroll:4 /Qfp-speculation=fast 
