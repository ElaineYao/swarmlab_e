% attack strategy 1
function x = attack(time)
%     disp("hi:");
    if ((time>21)&&(time <24))
        % decide the drones for attack
       fileid = fopen('/media/EDrive/swarmlab_e/flag.txt', 'w'); 
       drones = [1 0 1 0 0].';
       fprintf(fileid, '%d\n', drones);
       fclose(fileid);
       
%        % decide the noise
%        % for the 1st red one, keeps moving forward
%        fid = fopen('/media/EDrive/swarmlab_e/n1.txt', 'w'); 
%        n1 = [0.1 0 0].';
%        fprintf(fid, '%f\n', n1);
%        fclose(fid);
%        x = 1;
       
       % decide the noise
       % for the 1st red one, keep going to obstacle
       fid = fopen('/media/EDrive/swarmlab_e/n1.txt', 'w'); 
       n1 = [0.019 -0.015 0].';
       fprintf(fid, '%f\n', n1);
       fclose(fid);
       % for the 3rd green one, keep going to obstacle
       fid = fopen('/media/EDrive/swarmlab_e/n3.txt', 'w'); 
       n1 = [0.014 0.003 0].';
       fprintf(fid, '%f\n', n1);
       fclose(fid);
       x = 1;
    end
    
    
%     if ((time>24)&&(time <24.1))
%         % decide the drones for attack
%        fileid = fopen('/media/EDrive/swarmlab_e/flag.txt', 'w'); 
%        drones = [0 0 0 0 0].';
%        fprintf(fileid, '%d\n', drones);
%        fclose(fileid);
%     end
    
    if ((time>24)&&(time <28))
        % decide the drones for attack
       fileid = fopen('/media/EDrive/swarmlab_e/flag.txt', 'w'); 
       drones = [1 0 1 0 0].';
       fprintf(fileid, '%d\n', drones);
       fclose(fileid);
       
       fid = fopen('/media/EDrive/swarmlab_e/n1.txt', 'w'); 
       n1 = [0.007 0.008 0].';
       fprintf(fid, '%f\n', n1);
       fclose(fid);
       % for the 3rd green one, keep going to obstacle
       fid = fopen('/media/EDrive/swarmlab_e/n3.txt', 'w'); 
       n3 = [0.001 -0.013 0].';
       fprintf(fid, '%f\n', n3);
       fclose(fid);
       x = 1;
       
    end
    
    if ((time>28)&&(time <32))
        % decide the drones for attack
       fileid = fopen('/media/EDrive/swarmlab_e/flag.txt', 'w'); 
       drones = [1 0 1 0 0].';
       fprintf(fileid, '%d\n', drones);
       fclose(fileid);
       
       fid = fopen('/media/EDrive/swarmlab_e/n1.txt', 'w'); 
       n1 = [0.001 0.015 0].';
       fprintf(fid, '%f\n', n1);
       fclose(fid);
       % for the 3rd green one, keep going to obstacle
       fid = fopen('/media/EDrive/swarmlab_e/n3.txt', 'w'); 
       n3 = [0.0005 -0.017 0].';
       fprintf(fid, '%f\n', n3);
       fclose(fid);
       x = 1;
       
    end
    
 
end
