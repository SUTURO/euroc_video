:- module(testrobocop,
    [numberOfObjects/1,
    objectsFound/1,
    getInfoToDesig/3,
    getPutDownActions/1,
    getTimeForAction/3,
    objectsFoundAsList/1,
    objectsPlacedAsList/1,
    objectsGraspedAsList/1,
    placedObjectsAsList/1]).


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

getPutDownActions(D):-
	rdf_has(A, knowrob:taskSuccess, _),
	rdf_has(A,knowrob:designator,D), 
	getInfoToDesig(D,'TO', 'PUT-DOWN').

getGraspActions(D):-
	rdf_has(A, knowrob:taskSuccess, _),
	rdf_has(A,knowrob:designator,D), 
	getInfoToDesig(D,'TO', 'GRASP').

getTimeForAction(Action, Start, End):-
	rdf_has(D,knowrob:designator,Action),
	rdf_has(D,knowrob:startTime, Start),
	rdf_has(D,knowrob:endTime, End).


getPlacedObjects(Objects):-
	getPutDownActions(Actions),
	getObjectActedOn(Actions, Objects).

getGraspedObjects(Objects):-
	getGraspActions(Actions),
	getObjectActedOn(Actions, Objects).

getObjectActedOn(Action, Obj):-
	getInfoToDesig(Action,'OBJ.TYPE', Obj).


objectsFoundAsList(Result):-
	allObjectsFound(List),
	list_to_set(List, Result).

objectsPlacedAsList(Objects):-
	findall((Object,Start, End),
		(getPutDownActions(Action),
			getObjectActedOn(Action, Object),
			getTimeForAction(Action, Start, End)),
		Objects).

objectsGraspedAsList(Objects):-
	findall((Object,Start, End),
		(getGraspActions(Action),
			getObjectActedOn(Action, Object),
			getTimeForAction(Action, Start, End)),
		Objects).

placedObjectsAsList(Objects):-
	findall((Object, Start, End, X, Y, Z),
	(placedObjectInfos(Object, Start, End, X, Y, Z)),
	Objects).

placedObjectInfos(Object, S, E, X, Y, Z):-
	getPutDownActions(Action),
	getObjectActedOn(Action, Object),
	getTimeForAction(Action, S, E),
	mang_get_action_destination(Action, X, Y, Z).

