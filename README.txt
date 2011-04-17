 ========================================================================================
|											 |
|         SIMULATION OF A BIPED ROBOT WITH 6 DEGREES OF FREEDOM IN EACH LEG		 |
|         -----------------------------------------------------------------		 |
|           Oscar Efrain Ramos Ponce							 |
|	    Programa de Pos Graduacao em Engenharia Eletrica				 |
|           Universidade Federal do Rio Grande do Sul, Brasil 2011			 |
|									 		 |
 ========================================================================================

The folders that are included are the following:
    - src:     Contains the source code (*.cpp)
    - include: Contains the headers (*.h)
    - tests:   Files to test the functions (*.cpp), each file contains a main()
    - build:   Contains the Makefile to compile the source code and the tests

The folder 'build' already contains an executable of the test file named 
'robSimpleSteps.cpp', that tests 4 simple steps, which are precomputed 
before the start of the simulation ("offline" process).

There are other tests for some parts specific parts of the source code. To compile them, 
the Makefile in the 'build' folder has to be manually changed.

