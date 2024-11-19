function drawArm(T,figH,opt1,opt2)
    % drawArm Plots a 3D representation of an arm based on transformation matrices.
    %   T: Structure containing transformation matrices T10, T20, T30, T40.
    %   figH: Handle to the figure window where the arm will be drawn.
persistent lastPlot fcalls tr;
n_buffer = 50;

if nargin > 4
    error('requires at most 2 optional inputs');
end

switch nargin
    case 2
        opt1 = "hold_false";
        opt2 = "trace_false";
    case 3
        opt2 = "trace_false";
end
    
figure(figH);


if isempty(fcalls) 
    % Code to execute only on the first call
    fcalls = 1;
    xlim([0 0.2]);
    ylim([-0.1 0.1]);
    zlim([0 0.2]);
    view([-45 45]);
    axis square;
    set(gcf,'Visible','on')
    grid on
    if(opt2 == "trace_true")
        tr = gobjects(1,n_buffer);
    end
else
    fcalls = fcalls + 1;
end

if(~isempty(lastPlot) && opt1 == "hold_false")
    delete(lastPlot);
end

hold on
X = [0,T.T10(1,4),T.T20(1,4),T.T30(1,4),T.T40(1,4)];
Y = [0,T.T10(2,4),T.T20(2,4),T.T30(2,4),T.T40(2,4)];
Z = [0,T.T10(3,4),T.T20(3,4),T.T30(3,4),T.T40(3,4)];

if(opt1 == "hold_true")
    lastPlot = plot3(X,Y,Z,'LineWidth',2, 'Color', rand(1, 3));
else
    lastPlot = plot3(X,Y,Z,LineWidth=2,Color='k');
end

if(opt2 == "trace_true")
    delete(tr(1));
    tr = [tr(2:end) plot3(T.T40(1,4),T.T40(2,4),T.T40(3,4),'.',Color='r')];
end

drawnow limitrate
hold off

end

