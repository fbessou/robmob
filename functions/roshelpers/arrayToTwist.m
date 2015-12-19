function [ twist ] = arrayToTwist( array )
twist = createTwist();
twist.Linear.X = array(1);
twist.Linear.Y = array(2);
twist.Linear.Z = array(3);
twist.Angular.X = array(4);
twist.Angular.Y = array(5);
twist.Angular.Z = array(6);
end

