%
% =================================================================
%% Estimate3DPose estimate 5DoF pose from 2D orthographic silhouettes
%
% INPUT:
% modelPcl - [3 x M] pointcloud representing the shape, pereferably with very large M (>10000)
% silhouetteImage - [2 x L] 2D point sequence describing obtained orthographic silhouette
%    (<silhouetteImage> can either be 2D points on the closed contour or a semi-dense ...
%                                           ... pointcloud in the interior of the silhouette)
% shapeSignature - input shape signature, with the following sub-fields:
%                           - shapeSignature.areaProfile - [N x 1]
%                           - shapeSignature.perimeterProfile - [N x 1]
%                           - shapeSignature.postelPts - [N x 2]
% min3DMatches - [OPTIONAL] a scalar that defines the minimum no. of 3D matches (between area and 
%                               arc-length) required in the iso-contours
%                               for pose estimation [DEFAULT = 50]
% -----------------------------------------------------------------
% OUTPUT:
% pose - pose.rot is a [3x3] rotation matrix, pose.trans is a [2 x 1] translation vector
% objectRotEst - aligned/rotated object [3 x M]
% errorMatch - scalar representing final Hausdorff dist. between estimated and input silhouette
% timeElapsed - time taken for the estimation, in seconds
% =================================================================
%

function [pose, objectRotEst, errorMatch, timeElapsed, settings] = Estimate3DPose(modelPcl, silhouetteImage, shapeSignature, min3DMatches, ...
    Ncontour_P, posematchThreshold2D_P, posematchThreshold3D_P, resolution2DSearch_P)

    % Donot use <GT_ROT> for anything apart from debug
    %% Assertions
    assert(size(modelPcl,1) == 3, '<modelPcl> must be [3 x M]');
    assert(size(silhouetteImage,1) == 2, '<silhouetteImage> must be [2 x L]');
    assert(isfield(shapeSignature, 'areaProfile'), '<shapeSignature> must have a [N x 1] field <areaProfile>');
    assert(isfield(shapeSignature, 'postelPts'), '<shapeSignature> must have a [N x 2] field <postelPts>');
    assert(size(shapeSignature.areaProfile,1) == size(shapeSignature.postelPts,1));
    assert(size(shapeSignature.postelPts,2) == 2, '<shapeSignature> must have a [N x 2] field <postelPts>');
    if(size(modelPcl,2)<10000)
        warning('Input shape pointcloud is very sparse, may result in erroneous pose estimation! Please try to provide denser pointcloud, if possible.');
    end

    if exist('min3DMatches', 'var')
        assert(numel(min3DMatches) == 1, '<min3DMatches> should be a scalar');
        assert(min3DMatches > 3, 'Try with at least 3 matches!');
    end

    warning('off', 'all');
    
    %% Initializations

    posematchThreshold2D = 0.05; resolution2DSearch = 80; 
    posematchThreshold3D = 0.08; Ncontour = 150;
    minimumNumberOf3DMatchesRequired = 50;
    ALERT_THRESH = 0.5; % if computed signature is beyond the learned signature by this amount, create a ruckus!
    if exist('min3DMatches', 'var')
        minimumNumberOf3DMatchesRequired = min3DMatches;
    end
    if exist('Ncontour_P', 'var')
        Ncontour = Ncontour_P;
    end
    if exist('posematchThreshold2D_P', 'var')
        posematchThreshold2D = posematchThreshold2D_P;
    end
    if exist('posematchThreshold3D_P', 'var')
        posematchThreshold3D = posematchThreshold3D_P;
    end
    if exist('resolution2DSearch_P', 'var')
        resolution2DSearch = resolution2DSearch_P;
    end

    settings.posematchThreshold2D = posematchThreshold2D;
    settings.resolution2DSearch = resolution2DSearch;
    settings.posematchThreshold3D = posematchThreshold3D;
    settings.Ncontour = Ncontour;
    settings.minimumNumberOf3DMatchesRequired = minimumNumberOf3DMatchesRequired;

    startTime = tic; % ---: starting timer at this point

    %% Input silhouette processing
    [shp, area] = boundary(silhouetteImage(1, :).', silhouetteImage(2, :).');
    areaProj = area;    
    [perimeterProj] = getPerimeter(silhouetteImage(:, shp));
    
    [ellipse] = fit_ellipse(silhouetteImage(1, :).', silhouetteImage(2, :).');
    axisRatio = ellipse.short_axis/ellipse.long_axis;

    %% Testing intersections
    xv = linspace(-pi, pi, Ncontour);
    yv = linspace(-pi, pi, Ncontour);
    [X,Y] = meshgrid(xv, yv);
    % Creating grid structure from R -> R^2 mapping
    Zellipse = griddata(shapeSignature.postelPts(:,1), shapeSignature.postelPts(:,2), shapeSignature.ellipseAxisRatio,X,Y);
    Zarea = griddata(shapeSignature.postelPts(:,1), shapeSignature.postelPts(:,2), shapeSignature.areaProfile,X,Y);

    % Conditioning data:
    if((areaProj - max(max(Zarea))) > ALERT_THRESH)
        warning('Observed area signature significantly beyond the learned signature!');
    elseif((areaProj - max(max(Zarea))) > 0)
        areaProj = max(max(Zarea)) - 0.001;
    end

    if((areaProj - min(min(Zarea))) < -ALERT_THRESH)
        warning('Observed area signature significantly below the learned signature!');
    elseif((areaProj - min(min(Zarea))) < 0)
        areaProj = min(min(Zarea)) + 0.001;
    end

    if((axisRatio - max(max(Zellipse))) > ALERT_THRESH)
        warning('Observed ellipse-axis-ratio signature significantly beyond the learned signature!');
    elseif((axisRatio - max(max(Zellipse))) > 0)
        axisRatio = max(max(Zellipse)) - 0.001;
    end

    if((axisRatio - min(min(Zellipse))) < -ALERT_THRESH)
        warning('Observed ellipse-axis-ratio signature significantly below the learned signature!');
    elseif((axisRatio - min(min(Zellipse))) < 0)
        axisRatio = min(min(Zellipse)) + 0.001;
    end    

    % Cperim = contour(X, Y, Zperim,[perimeterProj perimeterProj], '-b', 'LineWidth',1.5); hold on;
    Cellipse = contour(X, Y, Zellipse,[axisRatio axisRatio], '-b', 'LineWidth',2.5); hold on;
    Cperim = Cellipse;
    Carea = contour(X, Y, Zarea,[areaProj areaProj], '--k', 'LineWidth',2.5); hold on;
    close all;

    if ((size(Cperim,2)>0) && (size(Carea,2)>0))
        % If both area and perimeter have valid isocontours
        contourTablePerim = getContourLineCoordinates(Cperim);
        contourTableArea = getContourLineCoordinates(Carea);

        close all;
        matchedPoints = 0;

        % Finding nearest neighbors
        [~, dist] = kNearestNeighbors([contourTablePerim.X contourTablePerim.Y], [contourTableArea.X,contourTableArea.Y], 1);

        while( (matchedPoints <= minimumNumberOf3DMatchesRequired) && (matchedPoints ~= numel(dist)))
            % Detecting intersections by nearest neighbors
            acceptedNeighbors = dist < posematchThreshold3D;

            matchedPoints = sum(acceptedNeighbors);
            
            % Incrementing threshold if not enough matches found:
            if(matchedPoints <= minimumNumberOf3DMatchesRequired)
                posematchThreshold3D = posematchThreshold3D + 0.01;
            end
        end

        xi_ = contourTableArea.X(acceptedNeighbors);
        yi_ = contourTableArea.Y(acceptedNeighbors);

    elseif(size(Cperim,2)>0)
        % If only ellipse-aspect-ratio have valid isocontour
        contourTablePerim = getContourLineCoordinates(Cperim);
        xi_ = contourTablePerim.X;
        yi_ = contourTablePerim.Y;
    elseif(size(Carea,2)>0)
        % If only area have valid isocontour
        contourTableArea = getContourLineCoordinates(Carea);
        xi_ = contourTableArea.X;
        yi_ = contourTableArea.Y;
    else
        % No one has valid isocontours; this cannot be tolerated, exiting sans output:
        error('No intersections detected!');
    end

    % Mapping intersection pt.s back to Postel disc ...
    [alpha, rVector] = unitDiscToPostel(xi_, yi_);

    numCandidates = numel(alpha);

    Rlist = zeros(numCandidates, 3, 3);

    for ii = 1:numCandidates
        % ... and Postel disc back to rotation matrices
        [R, ~] = postelToRotationMatrix(alpha(ii), rVector(ii,:).');  
        Rlist(ii, :, :) = R;
    end

    globalPoseIndex = 1;

    %% Pose matches in 2D
    while(globalPoseIndex == 1)
        for ii = 1:numCandidates   
            [poseList] = Estimate2DPose(modelPcl, squeeze(Rlist(ii, :, :)), silhouetteImage, resolution2DSearch, posematchThreshold2D);

            for jj = 1:size(poseList,2)
                globalPoseList{globalPoseIndex} = poseList{jj};
                globalPoseIndex = globalPoseIndex + 1;
            end
        end

        if(globalPoseIndex == 1)
            posematchThreshold2D = posematchThreshold2D + 0.01;
        end
    end

    %% Pose matches in 3D - with Hausdorff distance    
    kInp = boundary(silhouetteImage(1,:).',silhouetteImage(2,:).');
    silhouetteImgBoundary = silhouetteImage(:, kInp).';
    errList = zeros(1, globalPoseIndex-1);
    centroidSilhouette = mean(silhouetteImage,2);

    settings.numCandidatesTested = (globalPoseIndex-1);

    parfor ii = 1:(globalPoseIndex-1)        
        objectRotEst = globalPoseList{ii}*modelPcl;        
        orthProjEst = objectRotEst(1:2,:);
        orthProjEstTranslated = orthProjEst + (centroidSilhouette - mean(orthProjEst,2));        
        kEst = boundary(orthProjEstTranslated(1,:).',orthProjEstTranslated(2,:).');
        [hd] = HausdorffDist(orthProjEstTranslated(:, kEst).', silhouetteImgBoundary); 
        
        errList(1, ii) = hd;
    end

    %% Extracting best solution

    [errorMatch, bestMatch] = min(errList);
    
    objectRotEst = globalPoseList{bestMatch}*modelPcl;
    objectTransEst = (centroidSilhouette - mean(objectRotEst(1:2,:),2));
    objectRotEst = objectRotEst + [objectTransEst; 0];

    pose.rot = globalPoseList{bestMatch};
    pose.trans = objectTransEst;

    timeElapsed = toc(startTime); % ---: stopping timer at this point

    Re = round(objectRotEst,2);

end