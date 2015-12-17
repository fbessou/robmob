function [array] = twistToArray(twist)

array = [twist.Linear.X twist.Linear.Y twist.Linear.Z
         twist.Angular.X twist.Angular.Y twist.Angular.Z];
end

