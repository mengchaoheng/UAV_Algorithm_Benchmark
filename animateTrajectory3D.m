function animateTrajectory3D(time, log, par, traj)
% Animate actual and reference 3D trajectories after simulation.

    p = nedToPlot(log.p);
    pd = nedToPlot(log.pd);
    [poseP, poseR] = selectPoseLog(log, par.poseSource);

    fig = figure('Name','3D trajectory animation');
    ax = axes('Parent', fig);
    hold(ax, 'on');

    hPath = plot3(ax, pd(1,:), pd(2,:), pd(3,:), ':', ...
        'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
    hP = plot3(ax, nan, nan, nan, 'b', 'LineWidth', 1.8);
    hPd = plot3(ax, nan, nan, nan, '--', 'Color', [0.85 0.33 0.10], ...
        'LineWidth', 1.3);
    hPoint = plot3(ax, p(1,1), p(2,1), p(3,1), 'bo', ...
        'MarkerFaceColor', 'b', 'MarkerSize', 6);

    [origin, xB, yB, zB] = bodyAxes(poseP, poseR, 1, par.bodyAxisScale);
    hx = quiver3(ax, origin(1), origin(2), origin(3), xB(1), xB(2), xB(3), 0, 'r');
    hy = quiver3(ax, origin(1), origin(2), origin(3), yB(1), yB(2), yB(3), 0, 'g');
    hz = quiver3(ax, origin(1), origin(2), origin(3), zB(1), zB(2), zB(3), 0, 'b');

    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 35, 25);
    set(ax, 'ZDir', 'reverse');
    xlabel(ax, 'x_{NED} north (m)');
    ylabel(ax, 'y_{NED} east (m)');
    zlabel(ax, 'z_{NED} down (m)');
    setLimits(ax, [p, pd]);
    drawWorldNEDAxes(ax);
    title(ax, "3D trajectory animation: " + traj.name);
    legend(ax, [hPath, hP, hPd, hPoint, hx, hy, hz], ...
        {'reference path','actual','reference','vehicle','x_B','y_B','z_B'}, ...
        'Location','best');

    idxList = unique([1:max(1, round(par.animationFrameDt/par.dt)):numel(time), numel(time)]);
    speed = max(par.animationSpeed, eps);
    t0 = tic;

    for idx = idxList
        if ~ishandle(fig)
            break;
        end

        pause(max(0, (time(idx) - time(1))/speed - toc(t0)));

        set(hP, 'XData', p(1,1:idx), 'YData', p(2,1:idx), 'ZData', p(3,1:idx));
        set(hPd, 'XData', pd(1,1:idx), 'YData', pd(2,1:idx), 'ZData', pd(3,1:idx));
        set(hPoint, 'XData', p(1,idx), 'YData', p(2,idx), 'ZData', p(3,idx));

        [origin, xB, yB, zB] = bodyAxes(poseP, poseR, idx, par.bodyAxisScale);
        updateQuiver(hx, origin, xB);
        updateQuiver(hy, origin, yB);
        updateQuiver(hz, origin, zB);

        drawnow limitrate;
    end
end

function pPlot = nedToPlot(pNED)
    pPlot = pNED;
end

function [poseP, poseR] = selectPoseLog(log, poseSource)
    if poseSource == "desired"
        poseP = log.pd;
        poseR = log.Rd;
    else
        poseP = log.p;
        poseR = log.R;
    end
end

function [origin, xB, yB, zB] = bodyAxes(pLog, RLog, idx, scale)
    origin = nedToPlot(pLog(:,idx));
    R = RLog(:,:,idx);
    xB = scale*nedToPlot(R(:,1));
    yB = scale*nedToPlot(R(:,2));
    zB = scale*nedToPlot(R(:,3));
end

function updateQuiver(h, origin, vec)
    set(h, 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
        'UData', vec(1), 'VData', vec(2), 'WData', vec(3));
end

function setLimits(ax, pAll)
    span = max(max(pAll, [], 2) - min(pAll, [], 2));
    margin = 0.1*max(span, 1);
    xlim(ax, [min(pAll(1,:))-margin, max(pAll(1,:))+margin]);
    ylim(ax, [min(pAll(2,:))-margin, max(pAll(2,:))+margin]);
    zlim(ax, [min(pAll(3,:))-margin, max(pAll(3,:))+margin]);
end

function drawWorldNEDAxes(ax)
    xl = xlim(ax);
    yl = ylim(ax);
    zl = zlim(ax);

    dx = diff(xl);
    dy = diff(yl);
    dz = diff(zl);
    L = 0.14*max([dx, dy, dz]);

    origin = [xl(1) + 0.08*dx;
              yl(1) + 0.10*dy;
              zl(1) + 0.12*dz];

    quiver3(ax, origin(1), origin(2), origin(3), L, 0, 0, ...
        0, 'Color', [0.65 0 0], 'LineWidth', 1.5, ...
        'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
    quiver3(ax, origin(1), origin(2), origin(3), 0, L, 0, ...
        0, 'Color', [0 0.45 0], 'LineWidth', 1.5, ...
        'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
    quiver3(ax, origin(1), origin(2), origin(3), 0, 0, L, ...
        0, 'Color', [0 0.15 0.75], 'LineWidth', 1.8, ...
        'MaxHeadSize', 0.8, 'HandleVisibility', 'off');

    text(ax, origin(1)+1.10*L, origin(2), origin(3), '+x_N', ...
        'Color', [0.65 0 0], 'FontWeight', 'bold', ...
        'HandleVisibility', 'off');
    text(ax, origin(1), origin(2)+1.10*L, origin(3), '+y_E', ...
        'Color', [0 0.45 0], 'FontWeight', 'bold', ...
        'HandleVisibility', 'off');
    text(ax, origin(1), origin(2), origin(3)+1.10*L, '+z_D', ...
        'Color', [0 0.15 0.75], 'FontWeight', 'bold', ...
        'HandleVisibility', 'off');

    xlim(ax, xl);
    ylim(ax, yl);
    zlim(ax, zl);
end
