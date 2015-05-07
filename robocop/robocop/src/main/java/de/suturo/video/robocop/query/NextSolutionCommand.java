package de.suturo.video.robocop.query;

public class NextSolutionCommand extends QueryCommand {
    @Override
    public Object execute(jpl.Query query) {
        return query.nextElement();
    }
}
