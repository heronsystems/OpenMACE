function generateArducopterCmds( ROS_MACE , x , y )
    xx = x + ROS_MACE.XRef; 
    yy = y + ROS_MACE.YRef;    
    fid = fopen('arduCopter.sh','w');
    fprintf(fid,'#!/bin/bash\n');
    for i = 1:1:ROS_MACE.N
        [lat,long] = utm2deg(xx(i),yy(i),ROS_MACE.utmzone);
        fprintf(fid,'gnome-terminal -e "bash -c \\"sim_vehicle.py -I %d --out=udp:%s:1455%d --custom-location=%8.8f,%8.8f,0,240\\""\n',ROS_MACE.agentIDs(i),ROS_MACE.ip,ROS_MACE.agentIDs(i),lat,long);
    end
    fclose(fid);
    system('chmod +x arduCopter.sh');
end