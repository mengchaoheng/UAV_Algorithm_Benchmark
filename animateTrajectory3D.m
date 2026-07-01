function fig = animateTrajectory3D(time, log, par, ~)
% Animate actual and reference 3D trajectories after simulation.

    p = log.p;
    pd = log.pd;
    poseSource = getStructFieldLocal(par, 'animationPoseSource', "actual");
    bodyAxisScale = getStructFieldLocal(par, 'animationBodyAxisScale', 0.3);
    [poseP, poseR] = selectPoseLog(log, poseSource);

    fig = figure('Color', 'w', 'Name', 'anim_3d');
    ax = axes('Parent', fig);
    hold(ax, 'on');

    hPath = plot3(ax, pd(1,:), pd(2,:), pd(3,:), ':', ...
        'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
    hP = animatedline(ax, 'Color', 'b', 'LineWidth', 1.8);
    hPd = animatedline(ax, 'LineStyle', '--', ...
        'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
    hPoint = plot3(ax, p(1,1), p(2,1), p(3,1), 'bo', ...
        'MarkerFaceColor', 'b', 'MarkerSize', 6);
    addpoints(hP, p(1,1), p(2,1), p(3,1));
    addpoints(hPd, pd(1,1), pd(2,1), pd(3,1));

    [origin, xB, yB, zB] = bodyAxes(poseP, poseR, 1, bodyAxisScale);
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
    title(ax, "anim_3d", 'Interpreter', 'none');
    legend(ax, [hPath, hP, hPd, hPoint, hx, hy, hz], ...
        {'reference path','actual','reference','vehicle','x_B','y_B','z_B'}, ...
        'Location','best', 'Interpreter', 'none');

    speed = max(par.animationSpeed, eps);
    frameDt = max(par.animationFrameDt, eps);
    [frameTimes, idxList] = animationFrameIndices(time, frameDt);
    lastIdx = 1;

    drawnow nocallbacks;
    t0 = tic;

    for iFrame = 1:numel(idxList)
        pauseUntilFrame(t0, (frameTimes(iFrame) - time(1))/speed);

        if ~ishandle(fig)
            break;
        end

        idx = idxList(iFrame);
        if idx > lastIdx
            addpoints(hP, p(1,lastIdx+1:idx), ...
                p(2,lastIdx+1:idx), p(3,lastIdx+1:idx));
            addpoints(hPd, pd(1,lastIdx+1:idx), ...
                pd(2,lastIdx+1:idx), pd(3,lastIdx+1:idx));
            lastIdx = idx;
        end
        set(hPoint, 'XData', p(1,idx), 'YData', p(2,idx), 'ZData', p(3,idx));

        [origin, xB, yB, zB] = bodyAxes(poseP, poseR, idx, bodyAxisScale);
        updateQuiver(hx, origin, xB);
        updateQuiver(hy, origin, yB);
        updateQuiver(hz, origin, zB);

        drawnow nocallbacks;
    end
end

function [frameTimes, idxList] = animationFrameIndices(time, frameDt)
    time = time(:).';
    frameTimes = time(1):frameDt:time(end);
    if isempty(frameTimes) || frameTimes(end) < time(end)
        frameTimes(end+1) = time(end);
    end

    idxList = zeros(size(frameTimes));
    idx = 1;
    for i = 1:numel(frameTimes)
        while idx < numel(time) && time(idx+1) <= frameTimes(i)
            idx = idx + 1;
        end
        idxList(i) = idx;
    end

    [idxList, uniqueIdx] = unique(idxList, 'stable');
    frameTimes = frameTimes(uniqueIdx);
end

function pauseUntilFrame(t0, targetElapsed)
    remaining = targetElapsed - toc(t0);
    if remaining > 0
        pause(remaining);
    end
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
    origin = pLog(:,idx);
    R = RLog(:,:,idx);
    xB = scale*R(:,1);
    yB = scale*R(:,2);
    zB = scale*R(:,3);
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

function value = getStructFieldLocal(s, fieldName, defaultValue)
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
