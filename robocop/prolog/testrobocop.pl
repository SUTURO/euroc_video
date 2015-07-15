:- module(testrobocop,
    [numberFound/1,
    objectsFound/1,
    getInfoToDesig/3]).


:- load_experiment('/home/suturo/sr_experimental_data/current-experiment/cram_log.owl').

numberOfObjects(Count):-
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


getInfoToDesig(Desig, Attr, RESULT):-
	rdf_split_url(_, E, Desig),
        atom_concat('http://knowrob.org/kb/knowrob.owl#',E,Des),
        mang_designator_props(Des, Attr, RESULT).

getParkActions(D):-
	rdf_has(A, knowrob:taskSuccess, _),
	rdf_has(A,knowrob:designator,D), 
	getInfoToDesig(D,'TO', 'PARK').

getParkedObjects(Objects):-
	getParkActions(Actions),
	getInfoToDesig(Actions,'OBJ.TYPE', Objects).
