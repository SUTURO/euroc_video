package de.suturo.video.robocop.query;

/**
 * Abstract command for JPL queries.
 * 
 * @author Daniel Beßler
 */
public abstract class QueryCommand {
    public Object result = null;

    public abstract Object execute(jpl.Query query);
}
