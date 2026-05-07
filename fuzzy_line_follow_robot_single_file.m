function fuzzy_line_follow_robot_single_file(tuning)
% FUZZY_LINE_FOLLOW_ROBOT_SINGLE_FILE
% Single-file MATLAB implementation of a fuzzy-logic line-following robot.
% Run this function directly.

    if nargin < 1
        tuning = struct();
    end

    if ~isstruct(tuning)
        tuning = struct();
    end

    robot = createRobotParams(tuning);
    sim   = createSimulationParams(tuning);
    fis   = createFuzzyController();

    result = runSimulation(robot, sim, fis);
    plotResults(result, sim, robot);
end

function robot = createRobotParams(tuning)
    robot.v     = 0.25;   % Constant linear speed (m/s)
    robot.wMax  = 2.5;    % Max angular speed (rad/s)
    robot.eMax  = 0.50;   % Error normalization scale (m)
    robot.deMax = 2.00;   % Error-rate normalization scale (m/s)

    robot.x0     = 0.0;
    robot.y0     = -0.4;
    robot.theta0 = 0.0;

    robot.bodyLength = 0.22;   % Differential-drive body length (m)
    robot.bodyWidth  = 0.14;    % Differential-drive body width (m)
    robot.wheelLength = 0.09;   % Wheel visual length (m)
    robot.wheelWidth  = 0.03;    % Wheel visual width (m)

    robot = applyOverrides(robot, tuning, "robot");
end

function sim = createSimulationParams(tuning)
    sim.dt = 0.02;
    sim.T  = 24;
    sim.t  = 0:sim.dt:sim.T;

    % Reference line y = f(x)
    sim.lineFcn = @(x) 0.35*sin(0.8*x) + 0.05*cos(2.2*x);

    sim = applyOverrides(sim, tuning, "sim");

    if isfield(tuning, "lineFcn") && isa(tuning.lineFcn, "function_handle")
        sim.lineFcn = tuning.lineFcn;
    end

    sim.t = 0:sim.dt:sim.T;
end

function fis = createFuzzyController()
    fis = mamfis("Name","LineFollowerFIS");

    fis = addInput(fis,[-1 1],"Name","e");
    fis = addMF(fis,"e","trimf",[-1.0 -1.0 -0.5],"Name","NB");
    fis = addMF(fis,"e","trimf",[-1.0 -0.5  0.0],"Name","NS");
    fis = addMF(fis,"e","trimf",[-0.3  0.0  0.3],"Name","ZE");
    fis = addMF(fis,"e","trimf",[ 0.0  0.5  1.0],"Name","PS");
    fis = addMF(fis,"e","trimf",[ 0.5  1.0  1.0],"Name","PB");

    fis = addInput(fis,[-1 1],"Name","de");
    fis = addMF(fis,"de","trimf",[-1.0 -1.0 -0.5],"Name","NB");
    fis = addMF(fis,"de","trimf",[-1.0 -0.5  0.0],"Name","NS");
    fis = addMF(fis,"de","trimf",[-0.3  0.0  0.3],"Name","ZE");
    fis = addMF(fis,"de","trimf",[ 0.0  0.5  1.0],"Name","PS");
    fis = addMF(fis,"de","trimf",[ 0.5  1.0  1.0],"Name","PB");

    fis = addOutput(fis,[-1 1],"Name","w");
    fis = addMF(fis,"w","trimf",[-1.0 -1.0 -0.5],"Name","NB");
    fis = addMF(fis,"w","trimf",[-1.0 -0.5  0.0],"Name","NS");
    fis = addMF(fis,"w","trimf",[-0.3  0.0  0.3],"Name","ZE");
    fis = addMF(fis,"w","trimf",[ 0.0  0.5  1.0],"Name","PS");
    fis = addMF(fis,"w","trimf",[ 0.5  1.0  1.0],"Name","PB");

    % Rule format: [e de w weight AND_or_OR]
    % MF index order: NB=1, NS=2, ZE=3, PS=4, PB=5
    rules = [
        1 1 1 1 1; 1 2 1 1 1; 1 3 1 1 1; 1 4 2 1 1; 1 5 3 1 1;
        2 1 1 1 1; 2 2 1 1 1; 2 3 2 1 1; 2 4 3 1 1; 2 5 4 1 1;
        3 1 1 1 1; 3 2 2 1 1; 3 3 3 1 1; 3 4 4 1 1; 3 5 5 1 1;
        4 1 2 1 1; 4 2 3 1 1; 4 3 4 1 1; 4 4 5 1 1; 4 5 5 1 1;
        5 1 3 1 1; 5 2 4 1 1; 5 3 5 1 1; 5 4 5 1 1; 5 5 5 1 1
    ];
    fis = addRule(fis, rules);
end

function result = runSimulation(robot, sim, fis)
    N = numel(sim.t);

    x     = zeros(N,1);
    y     = zeros(N,1);
    theta = zeros(N,1);
    e     = zeros(N,1);
    de    = zeros(N,1);
    wCmd  = zeros(N,1);
    yRef  = zeros(N,1);

    x(1)     = robot.x0;
    y(1)     = robot.y0;
    theta(1) = robot.theta0;
    yRef(1)  = sim.lineFcn(x(1));
    e(1)     = yRef(1) - y(1);

    for k = 2:N
        yRef(k) = sim.lineFcn(x(k-1));
        e(k)    = yRef(k) - y(k-1);
        de(k)   = (e(k) - e(k-1)) / sim.dt;

        eN  = clamp(e(k)  / robot.eMax,  -1, 1);
        deN = clamp(de(k) / robot.deMax, -1, 1);

        wNorm   = evalfis(fis, [eN deN]);
        wCmd(k) = clamp(wNorm, -1, 1) * robot.wMax;

        theta(k) = wrapToPiLocal(theta(k-1) + wCmd(k)*sim.dt);
        x(k)     = x(k-1) + robot.v*cos(theta(k))*sim.dt;
        y(k)     = y(k-1) + robot.v*sin(theta(k))*sim.dt;
    end

    result.x     = x;
    result.y     = y;
    result.theta = theta;
    result.e     = e;
    result.de    = de;
    result.wCmd  = wCmd;
    result.yRef  = yRef;
    result.metrics.rmsError = sqrt(mean(e.^2));
    result.metrics.maxAbsError = max(abs(e));
    result.metrics.finalError = e(end);
end

function plotResults(result, sim, robot)
    xLine = linspace(min(result.x)-0.2, max(result.x)+0.2, 700);
    yLine = sim.lineFcn(xLine);

    figure("Color","w","Name","Fuzzy Line Follower");
    tiledlayout(3,1,"Padding","compact","TileSpacing","compact");

    nexttile;
    plot(xLine, yLine, "k--","LineWidth",1.6); hold on;
    plot(result.x, result.y, "b","LineWidth",1.6);
    trailLine = animatedline("Color",[0 0.45 0.74],"LineWidth",2.0,"HandleVisibility","off");
    [bodyPatch, leftWheelPatch, rightWheelPatch, headingLine] = createVehicleGraphics();
    axis equal; grid on;
    xlabel("x (m)"); ylabel("y (m)");
    title(sprintf("Robot Trajectory vs Reference Line | RMS e = %.3f m", result.metrics.rmsError));
    legend("Reference line","Robot path","Location","best");
    xlim([min(xLine) max(xLine)]);
    ylim([min(min(yLine), min(result.y)) - 0.2, max(max(yLine), max(result.y)) + 0.2]);

    for k = 1:numel(result.x)
        [bx, by, lwx, lwy, rwx, rwy, hx, hy] = vehicleGeometry(result.x(k), result.y(k), result.theta(k), robot);
        set(bodyPatch, "XData", bx, "YData", by);
        set(leftWheelPatch, "XData", lwx, "YData", lwy);
        set(rightWheelPatch, "XData", rwx, "YData", rwy);
        set(headingLine, "XData", hx, "YData", hy);
        addpoints(trailLine, result.x(k), result.y(k));
        drawnow limitrate;
    end

    nexttile;
    plot(sim.t, result.e, "r","LineWidth",1.5); grid on;
    xlabel("Time (s)"); ylabel("Error e (m)");
    title("Tracking Error");

    nexttile;
    plot(sim.t, result.wCmd, "m","LineWidth",1.5); grid on;
    xlabel("Time (s)"); ylabel("\omega command (rad/s)");
    title("Fuzzy Controller Output");
end

function params = applyOverrides(params, tuning, groupName)
    if isfield(tuning, groupName) && isstruct(tuning.(groupName))
        overrideFields = fieldnames(tuning.(groupName));
        for i = 1:numel(overrideFields)
            fieldName = overrideFields{i};
            params.(fieldName) = tuning.(groupName).(fieldName);
        end
    end
end

function [bodyX, bodyY, leftWheelX, leftWheelY, rightWheelX, rightWheelY, headingX, headingY] = vehicleGeometry(x, y, theta, robot)
    bodyLocal = [
         robot.bodyLength/2,  robot.bodyLength/2, -robot.bodyLength/2, -robot.bodyLength/2;
         robot.bodyWidth/2,  -robot.bodyWidth/2,  -robot.bodyWidth/2,   robot.bodyWidth/2
    ];

    wheelY = robot.bodyWidth/2 + robot.wheelWidth/2;
    wheelLocal = [
         -robot.wheelLength/2,  robot.wheelLength/2;
          wheelY,               wheelY
    ];
    wheelLocalMirror = wheelLocal;
    wheelLocalMirror(2,:) = -wheelLocalMirror(2,:);

    headingLocal = [0, robot.bodyLength/2; 0, 0];

    transform = [cos(theta) -sin(theta); sin(theta) cos(theta)];

    bodyPts = transform * bodyLocal + [x; y];
    leftPts = transform * wheelLocal + [x; y];
    rightPts = transform * wheelLocalMirror + [x; y];
    headingPts = transform * headingLocal + [x; y];

    bodyX = bodyPts(1,:);
    bodyY = bodyPts(2,:);
    leftWheelX = leftPts(1,:);
    leftWheelY = leftPts(2,:);
    rightWheelX = rightPts(1,:);
    rightWheelY = rightPts(2,:);
    headingX = headingPts(1,:);
    headingY = headingPts(2,:);
end

function [bodyPatch, leftWheelPatch, rightWheelPatch, headingLine] = createVehicleGraphics()
    bodyPatch = patch("XData",nan,"YData",nan,"FaceColor",[0.2 0.6 0.9],"FaceAlpha",0.35,"EdgeColor",[0.1 0.25 0.45],"LineWidth",1.5,"HandleVisibility","off");
    leftWheelPatch = patch("XData",nan,"YData",nan,"FaceColor",[0.12 0.12 0.12],"EdgeColor",[0.05 0.05 0.05],"LineWidth",1.0,"HandleVisibility","off");
    rightWheelPatch = patch("XData",nan,"YData",nan,"FaceColor",[0.12 0.12 0.12],"EdgeColor",[0.05 0.05 0.05],"LineWidth",1.0,"HandleVisibility","off");
    headingLine = plot([nan nan], [nan nan], "Color",[0.85 0.33 0.1],"LineWidth",2.0,"HandleVisibility","off");
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function ang = wrapToPiLocal(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
