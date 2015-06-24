:- module(testhelper, [
	getNotableTimepoints/1,
	addNotableTimepoint/1
   ]).

:- dynamic(notableTimepoints/1).
:- assert(notableTimepoints([])).


getNotableTimepoints(Timepoints) :-
	notableTimepoints(Timepoints).

addNotableTimepoint(Timepoint) :-
	notableTimepoints(List),
	append(List, [Timepoint], NewList),
	retractall(notableTimepoints(_)),
	assertz(notableTimepoints(NewList)).