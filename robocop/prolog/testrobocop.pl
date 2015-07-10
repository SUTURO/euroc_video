:- module(testrobocop,
    [numberFound/1,
    objectsFound/1]).


:- ensure_loaded('/home/suturo/catkin_ws/src/euroc_video/robocop/prolog/init.pl').
:- load_experiment('/home/suturo/sr_experimental_data/current-experiment/cram_log.owl').

numberFound(Count):-
	findall(RESULT,objectsFound(RESULT)
	, List),
	length(List, Count).

allObjectsFound(List):-
        findall(RESULT,(
	owl_has(_,knowrob:successorDesignator,A),
        rdf_split_url(_, E, A), 
        atom_concat('http://knowrob.org/kb/knowrob.owl#',E,Des),
        mang_designator_props(Des,'TYPE',RESULT)),
	List).

objectsFound(Result):-
	allObjectsFound(List),
	list_to_set(List, Set),
	member(Result,Set).



