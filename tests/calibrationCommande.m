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

nSamples = 10;
GAZ_MAP_TF = [5 12]';

initialPosition = [ 8 3 3*pi/4]';
initialCovariance = zeros(3,3);

dmin = 0.2;
dmax = 2;
step = 0.2;
nSteps = (dmax-dmin+step)/step;

effectiveDistances = zeros(nSteps,2);
effectiveDistances(:,1) = linspace(dmin,dmax,nSteps)';
for currentCommandId = 1:1:nSteps
    for sample = 1:1:10
        % place robot at starting point
        Pose = createPose(initialPosition - [GAZ_MAP_TF; 0]);
        gazeboForceModelState('mobile_base',Pose,createTwist());
        Cov = initialCovariance;
        
        currentCommand = dmin + step*currentCommandId;
        
        smoothWalk(currentCommand,vel_mux_publisher);
        [position Cov] = triangularLocalization(initialPosition,Cov,[currentCommand 0],imsub);
        delta = position(1:2)-initialPosition(1:2);
        distance = norm(delta);
        
        % weighted arithmetic mean of aggregated values
        effectiveDistances(currentCommandId,2)= (effectiveDistances(currentCommandId,2)*(sample-1)+distance)/sample;
    end
end
gazeboForceModelState('mobile_base',pose,createTwist());
