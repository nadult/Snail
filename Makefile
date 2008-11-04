#profile:
#	-rm callgrind.out* cachegrind.out*
#	valgrind --tool=callgrind --dump-line=yes --collect-jumps=yes --simulate-cache=yes ./rtracer -res 800 600 eagles_nest2.obj
#	kcachegrind

#profilec:
#	-rm callgrind.out* cachegrind.out*
#	alleyop ./rtracer -res 800 600 eagles_nest2.obj

prof:
	gprof rtracer|less -S

.PHONY: profile profilec
