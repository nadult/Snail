profile:
	-rm callgrind.out* cachegrind.out*
	valgrind --tool=callgrind ./rtracer 512 512 0 2 abrams.obj
	kcachegrind

profilec:
	-rm callgrind.out* cachegrind.out*
	valgrind --tool=cachegrind ./rtracer 512 512 0 2 abrams.obj
	kcachegrind

.PHONY: profile profilec
