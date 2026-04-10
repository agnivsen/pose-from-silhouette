function [poseList] = Estimate2DPose(modelPcl, initRotationMatrix, silhouetteImage, resolution, matchThreshold)

    assert(size(modelPcl,1) == 3, 'Input model has to be [3 x N] pointcloud');
    assert(size(initRotationMatrix,1) == 3, 'We need a [3 x 3] rotation matrix in SO(3)');
    assert(size(silhouetteImage,1) == 2, ['We need a [2 x M] pointcloud of 2D points inside the silhouette, ' ...
        'the pointcloud should be as dense as possible']);

    if ~exist('resolution', 'var')
        resolution = 100;
    end

    if ~exist('matchThreshold', 'var')
        matchThreshold = 0.01;
    end

    rotModel = initRotationMatrix * modelPcl;

    zSteps = linspace(0, 2*pi, resolution);

    lengthList = zeros(2, resolution);

    for ii = 1:resolution
        rotMat = rotz(rad2deg(zSteps(ii)));
        currModel = rotMat*rotModel;
        lengthCurr = range(currModel(1,:));
        lengthList(1,ii) = lengthCurr;
        lengthCurrY = range(currModel(2,:));
        lengthList(2,ii) = lengthCurrY;
    end

    inputLength = range(silhouetteImage(1,:));
    inputLengthY = range(silhouetteImage(2,:));

    matchVal = abs(lengthList(1,:) - inputLength) < matchThreshold;
    matchValY = abs(lengthList(2,:) - inputLengthY) < matchThreshold;

    filteredPoseIndex = 1;
    for ii = 1:resolution
        if(matchVal(ii) && matchValY(ii))
            rotMat1 = rotz(rad2deg(zSteps(ii)));
            rotMat2 = rotz(rad2deg(zSteps(ii) + pi));
            filteredPose{filteredPoseIndex} = rotMat1*initRotationMatrix;
            filteredPoseIndex = filteredPoseIndex + 1;
            % filteredPose{filteredPoseIndex} = rotMat2*initRotationMatrix;
            % filteredPoseIndex = filteredPoseIndex + 1;            
        end
    end

    if(filteredPoseIndex > 1)
        poseList = filteredPose;
    else
        poseList = [];
    end

end