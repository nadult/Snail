profile:
	-rm callgrind.out* cachegrind.out*
	valgrind --tool=callgrind ./rtracer 800 600 0 2
	kcachegrind

profilec:
	-rm callgrind.out* cachegrind.out*
	valgrind --tool=cachegrind ./rtracer 800 600 0 2
	kcachegrind

.PHONY: profile profilec
