:- module(testrobocop,
    [numberFound/1,
    objectsFound/1]).


:- ensure_loaded('/home/suturo/catkin_ws/src/euroc_video/robocop/prolog/init.pl').
:- load_experiment('/home/suturo/sr_experimental_data/current-experiment/cram_log.owl').

numberFound(Count):-
	findall(RESULT,objectsFound(RESULT)
	, List),
	length(List, Count).

objectsFound(RESULT):-
	owl_has(A,rdf:type,knowrob:'CRAMDesignator'),
        owl_has(A,knowrob:equatedDesignator,_),
        owl_has(A,knowrob:successorDesignator,_),
        rdf_split_url(_, E, A), 
        atom_concat('http://knowrob.org/kb/knowrob.owl#',E,Des),
        mng_designator_props(Des,'TYPE',RESULT).
