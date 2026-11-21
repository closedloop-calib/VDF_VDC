function data = readCalibrationData(filePath)
    % READCALIBRATIONDATA Read the tagged real-world calibration text file.
    fid = fopen(filePath, 'r');
    if fid == -1
        error('Cannot open data file: %s', filePath);
    end
    fileCleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    data = struct();
    data.cameraMatrix = [];
    data.worldPoints = [];
    data.imagePoints = {};
    data.baseInTcp = [];
    data.boardInCamera = [];
    tline = fgetl(fid);
    while ischar(tline)
        if contains(tline, '# KM')
            K = zeros(3, 4);
            for j = 1:3
                K(j, :) = fscanf(fid, '%f', [1, 4]);
            end
            data.cameraMatrix = K;
        elseif contains(tline, '# Worldpts')
            pts = [];
            while true
                pos = ftell(fid);
                tline2 = fgetl(fid);
                if ~ischar(tline2) || contains(tline2, '#')
                    fseek(fid, pos, -1);
                    break
                end
                vals = sscanf(tline2, '%f');
                if numel(vals) == 2
                    pts = [pts; vals'];
                end
            end
            data.worldPoints = pts;
        elseif contains(tline, '# imgPts')
            frames = {};
            framePts = [];
            while true
                pos = ftell(fid);
                tline2 = fgetl(fid);
                if ~ischar(tline2)
                    break
                elseif contains(tline2, '# Frame')
                    if ~isempty(framePts)
                        frames{end + 1} = framePts;
                    end
                    framePts = [];
                elseif contains(tline2, '#')
                    fseek(fid, pos, -1);
                    break
                else
                    vals = sscanf(tline2, '%f');
                    if numel(vals) == 2
                        framePts = [framePts; vals'];
                    end
                end
            end
            if ~isempty(framePts)
                frames{end + 1} = framePts;
            end
            Nframes = numel(frames);
            Np = size(frames{1}, 1);
            imgPtsArray = zeros(Np, 2, Nframes);
            for f = 1:Nframes
                imgPtsArray(:, :, f) = frames{f};
            end
            data.imagePoints = imgPtsArray;
        elseif contains(tline, '# BasePoseIntcpCoords')
            RobotPoses = [];
            tline = fgetl(fid);
            while ischar(tline) && ~contains(tline, '# gridPoseInCameraCoords')
                if contains(tline, '# RobotPose')
                    block = zeros(4, 4);
                    for j = 1:4
                        row = fgetl(fid);
                        if ischar(row)
                            block(j, :) = sscanf(row, '%f')';
                        end
                    end
                    if isempty(RobotPoses)
                        RobotPoses = block;
                    else
                        RobotPoses(:, :, end + 1) = block;
                    end
                end
                tline = fgetl(fid);
            end
            data.baseInTcp = RobotPoses;
            if ischar(tline) && contains(tline, '# gridPoseInCameraCoords')
                CameraPoses = [];
                tline = fgetl(fid);
                while ischar(tline)
                    if contains(tline, '# CameraPose')
                        block = zeros(4, 4);
                        for j = 1:4
                            row = fgetl(fid);
                            if ischar(row)
                                block(j, :) = sscanf(row, '%f')';
                            end
                        end
                        if isempty(CameraPoses)
                            CameraPoses = block;
                        else
                            CameraPoses(:, :, end + 1) = block;
                        end
                    end
                    tline = fgetl(fid);
                end
                data.boardInCamera = CameraPoses;
                break
            end
        end
        tline = fgetl(fid);
    end
end
