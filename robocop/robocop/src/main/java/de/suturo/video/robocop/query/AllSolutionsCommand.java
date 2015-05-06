package de.suturo.video.robocop.query;

public class AllSolutionsCommand extends QueryCommand {
    @Override
    public Object execute(jpl.Query query) {
        return query.allSolutions();
    }
}
