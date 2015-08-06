:- module(testhelper, [
	getNotableTimepoints/1,
	addNotableTimepoint/1,
	initializeNotableTimepoints/0
   ]).

:- dynamic(notableTimepoints/1).
:- assert(notableTimepoints([])).


getNotableTimepoints(Timepoints) :-
	notableTimepoints(Timepoints).

initializeNotableTimepoints :-
	retractall(notableTimepoints(_)),
	assertz(notableTimepoints([])).

addNotableTimepoint(Timepoint) :-
	notableTimepoints(List),
	append(List, [Timepoint], NewList),
	retractall(notableTimepoints(_)),
	assertz(notableTimepoints(NewList)).