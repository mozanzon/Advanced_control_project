function fuzzy_line_follow_robot_single_file()
% FUZZY_LINE_FOLLOW_ROBOT_SINGLE_FILE
% Single-file MATLAB implementation of a fuzzy-logic line-following robot.
% Run this function directly.

    robot = createRobotParams();
    sim   = createSimulationParams();
    fis   = createFuzzyController();

    result = runSimulation(robot, sim, fis);
    plotResults(result, sim);
end

function robot = createRobotParams()
    robot.v     = 0.25;   % Constant linear speed (m/s)
    robot.wMax  = 2.5;    % Max angular speed (rad/s)
    robot.eMax  = 0.50;   % Error normalization scale (m)
    robot.deMax = 2.00;   % Error-rate normalization scale (m/s)

    robot.x0     = 0.0;
    robot.y0     = -0.4;
    robot.theta0 = 0.0;
end

function sim = createSimulationParams()
    sim.dt = 0.02;
    sim.T  = 24;
    sim.t  = 0:sim.dt:sim.T;

    % Reference line y = f(x)
    sim.lineFcn = @(x) 0.35*sin(0.8*x) + 0.05*cos(2.2*x);
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
end

function plotResults(result, sim)
    xLine = linspace(min(result.x)-0.2, max(result.x)+0.2, 700);
    yLine = sim.lineFcn(xLine);

    figure("Color","w","Name","Fuzzy Line Follower");
    tiledlayout(3,1,"Padding","compact","TileSpacing","compact");

    nexttile;
    plot(xLine, yLine, "k--","LineWidth",1.6); hold on;
    plot(result.x, result.y, "b","LineWidth",2.0);
    axis equal; grid on;
    xlabel("x (m)"); ylabel("y (m)");
    title("Robot Trajectory vs Reference Line");
    legend("Reference line","Robot path","Location","best");

    nexttile;
    plot(sim.t, result.e, "r","LineWidth",1.5); grid on;
    xlabel("Time (s)"); ylabel("Error e (m)");
    title("Tracking Error");

    nexttile;
    plot(sim.t, result.wCmd, "m","LineWidth",1.5); grid on;
    xlabel("Time (s)"); ylabel("\omega command (rad/s)");
    title("Fuzzy Controller Output");
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function ang = wrapToPiLocal(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
