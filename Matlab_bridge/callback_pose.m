
function callback_pose(src,msg)


    global q R p

    
    try
        if (strcmp(msg.Transforms(1).ChildFrameId,'EspeleoRobo') == 1)
            
            %Get the robot position
            p(1) = msg.Transforms(1).Transform.Translation.X;
            p(2) = msg.Transforms(1).Transform.Translation.Y;
            p(3) = msg.Transforms(1).Transform.Translation.Z;

            %Get the robot orientation (quaternion)
            q(1) = msg.Transforms(1).Transform.Rotation.W;
            q(2) = msg.Transforms(1).Transform.Rotation.X;
            q(3) = msg.Transforms(1).Transform.Rotation.Y;
            q(4) = msg.Transforms(1).Transform.Rotation.Z;

            %Get a rotation matrix
            R = quat2rotm(q');
        end
    catch
        warning('Problem using function callback_pose');
    end
    
    
end %function