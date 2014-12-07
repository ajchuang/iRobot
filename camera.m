function [center_of_mass, region_of_interest] = camera ()
    image = imread ('http://192.168.0.101/img/snapshot.cgi?');
    retry_cnt = 0;
    while (true)
        try
            image = imread ('http://192.168.0.101/img/snapshot.cgi?');
            [BW, maskedRGBImage] = createMask (image);
            [x, y] = size (BW);
            imshow (BW)
            center_of_mass = COM(BW)
            region_of_interest = nnz (BW)
            return;
        catch
            retry_cnt = retry_cnt + 1;
            if (retry_cnt == 3)
                display ('too much');
                exit (0);
            end    
            display ('I can not take the picture.');
        end
    end
    
end