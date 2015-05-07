package de.suturo.video.robocop.query;

public class ResetCommand extends QueryCommand {
    @Override
    public Object execute(jpl.Query query) {
        query.rewind();
        return new Boolean(true);
    }
}
