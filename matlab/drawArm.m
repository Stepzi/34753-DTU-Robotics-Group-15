function drawArm(T,figH)
    % drawArm Plots a 3D representation of an arm based on transformation matrices.
    %   T: Structure containing transformation matrices T10, T20, T30, T40.
    %   figH: Handle to the figure window where the arm will be drawn.
persistent lastPlot fC;
    
figure(figH);

if isempty(fC)
    % Code to execute only on the first call
    fC = true;
    xlim([-0.2 0.2]);
    ylim([-0.2 0.2]);
    zlim([0 0.4]);
    view([-45 45]);
    axis square;
    set(gcf,'Visible','on')
    grid on
    

end

if(~isempty(lastPlot))
    delete(lastPlot);
end

hold on
X = [0,T.T10(1,4),T.T20(1,4),T.T30(1,4),T.T40(1,4)];
Y = [0,T.T10(2,4),T.T20(2,4),T.T30(2,4),T.T40(2,4)];
Z = [0,T.T10(3,4),T.T20(3,4),T.T30(3,4),T.T40(3,4)];
lastPlot = plot3(X,Y,Z,LineWidth=2,Color='k');
% plot3(T.T40(1,4),T.T40(2,4),T.T40(3,4),'+',Color='r');
drawnow limitrate
hold off

end

