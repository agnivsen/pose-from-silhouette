function [objectEst] = GlOptiPoS(object, orthProjInput, shapeSignature)
% GlOptiPoS - Globally Optimal Pose-from-Silhouettes
%
%
% INPUTS:
%   object           - Matrix representing the 3D coordinates of the object points
%   orthProjInput    - 2D orthogonal projection image or silhouette of the object
%   shapeSignature   - Structure containing precomputed features of the object shape:
%                      .areaProfile       - Profile of area along object
%                      .postelPts         - Key points (e.g., Postel points)
%                      .ellipseAxisRatio  - Ellipse axis ratio for shape approximation
%
% OUTPUT:
%   objectEst        - Matrix representing the estimated 3D object points after pose optimization


%% ----------------------- Initialization Section ------------------------
% Constants to control iteration and stopping criteria
ACCEPTED_HAUSDORFF = 0.1;    % Threshold of Hausdorff error below which the pose is considered good
MAX_ITER_EST = 100;           % Maximum number of pose estimation iterations

% Copy relevant fields from input shapeSignature to local structure
shapeSignatureC.areaProfile      = shapeSignature.areaProfile;
shapeSignatureC.postelPts        = shapeSignature.postelPts;
shapeSignatureC.ellipseAxisRatio = shapeSignature.ellipseAxisRatio;

% Initialize error metric to maximum possible real number
errorMatch = realmax;

% Iteration counter initialization
iterEst = 0;

% Default pose estimation parameters
poseThresh2DDefault = 0.01;    % 2D pose convergence threshold
poseThresh3DDefault = 0.12;    % 3D pose convergence threshold
res2DsearchDefault = 30;       % Resolution of 2D search grid for matching
nContourDefault = 150;         % Number of contour points used for matching
minMatchReqDefault = 18;       % Minimum required 2D matches for pose update
min3DMatches = 5;              % Minimum 3D points required for initial pose estimate

% Variables for adaptive threshold control
lastIterHausdorff = realmax;   % Stores Hausdorff error from previous iteration
diff2DIncrThresh = 0.001;      % Minimum change in Hausdorff error to adjust 2D threshold

%% --------------------- Iterative Pose Estimation Loop ------------------
while((iterEst < MAX_ITER_EST) && (errorMatch > ACCEPTED_HAUSDORFF))
    
    % ----------------- Pose Estimation Call -----------------
    % Estimate the 3D pose using current object, projection, and shape signature
    % Estimate3DPoseDebug is assumed to return:
    %   - pose: structure containing rotation matrix 'rot' and translation vector 'trans'
    %   - errorMatch: Hausdorff distance error between projected and observed silhouette
    [pose, ~, errorMatch, ~, ~] = Estimate3DPose(object, orthProjInput, shapeSignatureC, ...
        min3DMatches, nContourDefault, poseThresh2DDefault, poseThresh3DDefault, res2DsearchDefault);

    % ----------------- Apply Pose Transformation -----------------
    % Update the estimated object coordinates with the new pose
    objectEst = (pose.rot * object) + [pose.trans; 0];

    % Increment iteration counter
    iterEst = iterEst + 1;

    % ----------------- Adaptive Threshold Adjustment -----------------
    if(errorMatch > ACCEPTED_HAUSDORFF)
        % If current Hausdorff error is still too high, adaptively loosen constraints        
        min3DMatches = min3DMatches + 20;
        minMatchReqDefault = minMatchReqDefault + 3;
        res2DsearchDefault = res2DsearchDefault + 50;
        if(abs(errorMatch - lastIterHausdorff) < diff2DIncrThresh)
            poseThresh2DDefault = poseThresh2DDefault + 0.02;
        end
        poseThresh3DDefault = poseThresh3DDefault + 0.2;
        lastIterHausdorff = errorMatch;
    end
end

%% --------------------- End of Function --------------------------
% Return the final estimated object coordinates after iterative pose optimization
end
