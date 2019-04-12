function plotHandles = updateLogLRMapMovie(swarmWorld, swarmState, targetState, trueWorld, runParams, swarmModel, targetModel, plotHandles)
subplot(plotHandles.subplotHandle)
set(plotHandles.figh_subplot2,'CData', log( swarmWorld.O./swarmWorld.U ) );
end