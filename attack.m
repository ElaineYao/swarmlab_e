% attack strategy 1
function x = attack(time)
%     disp("hi:");
    if time>28
        % decide the drones for attack
       fileid = fopen('/media/EDrive/swarmlab_e/flag.txt', 'w'); 
       drones = [1 0 0 0 0].';
       fprintf(fileid, '%d\n', drones);
       fclose(fileid);
       
       % decide the noise
       % for the 1st red one, keeps moving forward
       fid = fopen('/media/EDrive/swarmlab_e/n1.txt', 'w'); 
       n1 = [0.1 0 0].';
       fprintf(fid, '%f\n', n1);
       fclose(fid);
       x = 1;
    end
end
