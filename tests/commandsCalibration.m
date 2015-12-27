%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';
%%


try
    %%
    rosinit(ipaddress);
    load('basic_world_graph')
    load('basic_world')
    vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
    imsub = rossubscriber('/camera/rgb/image_raw');
catch
end


GAZ_MAP_TF = [5 12]';



%% X translation command calibration
initialPosition = [ 8 3 3*pi/4]';
initialCovariance = zeros(3,3);

dmin = 0.1;
dmax = 2;
step = 0.1;
nSteps = (dmax-dmin+step)/step;

nSamples = 5;

effectiveDistances = zeros(nSteps,2);
effectiveDistances(:,1) = linspace(dmin,dmax,nSteps)';
for currentCommandId = 1:1:nSteps
    for sample = 1:1:nSamples
        % place robot at starting point
        Pose = createPose(initialPosition - [GAZ_MAP_TF; 0]);
        gazeboForceModelState('mobile_base',Pose,createTwist());
        Cov = ones(3,3);
        
        currentCommand = dmin + step*(currentCommandId-1);
        
        smoothWalk(currentCommand,vel_mux_publisher);
        [position Cov] = triangularLocalization(vel_mux_publisher,imsub,initialPosition,Cov,[currentCommand 0],zeros(1,3));
        delta = position(1:2)-initialPosition(1:2);
        distance = norm(delta);
        
        % weighted arithmetic mean of aggregated values
        effectiveDistances(currentCommandId,2)= (effectiveDistances(currentCommandId,2)*(sample-1)+distance)/sample;
    end
end
TranslationCommands = [effectiveDistances , ones(nSteps,1)*nSamples ];

%% Z rotation command calibration

initialPosition = [ 5 11 0]';
initialCovariance = zeros(3,3);

dmin = -pi;
dmax = pi;
step = pi/12;
nSteps = (dmax-dmin+step)/step;

nSamples = 2;

effectiveRotations = zeros(nSteps,2);
effectiveRotations(:,1) = linspace(dmin,dmax,nSteps)';
for currentCommandId = 1:1:nSteps
    for sample = 1:1:nSamples
        % place robot at starting point
        Pose = createPose(initialPosition - [GAZ_MAP_TF; 0]);
        gazeboForceModelState('mobile_base',Pose,createTwist());
        Cov = initialCovariance;
        
        currentCommand = dmin + step*(currentCommandId-1);
        if currentCommand~=0
            linearRotate(currentCommand,vel_mux_publisher);
            [position Cov] = gazeboLocalization();
            if currentCommandId < nSteps/2
                angle = mod(position(3) + 2*pi,2*pi)-2*pi;
            else
                angle = mod(position(3) + 2*pi,2*pi);
            end
            % weighted arithmetic mean of aggregated values
            effectiveRotations(currentCommandId,2)= (effectiveRotations(currentCommandId,2)*(sample-1)+angle)/sample;
        else
            effectiveRotations(currentCommandId,2) = 0;
        end
        
    end
end

RotationCommands = [effectiveRotations , ones(nSteps,1)*nSamples ];
linearised = polyfit(effectiveRotations(:,1),effectiveRotations(:,2),1);
RotationCommandFactor =  linearised(1)

